#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
云端连通性探测脚本 —— 完全复刻 Echo2 固件的 WebSocket 上行契约。

用途：把「固件音频到底有没有真正到达云端 / 云端会不会回识别结果」这件事，
      从真机里独立出来，用 PC 直连云端复现，一次性判断问题在固件还是云端。

复刻的固件契约（源自 main/protocol/websocket_client.c、main/session/session.c）：
  1) 连接 URL：  wss://<host>/ws/omni?token=<accessToken>
  2) 握手头：    Device-Id / Client-Id / Authorization: Bearer <token> / Protocol-Version: 1
  3) 握手文本帧：{"type":"start","format":"opus","sampleRate":16000,"deviceId":"<id>"}
  4) 音频：      纯二进制 OPUS 帧（16kHz / 单声道 / 60ms/帧），无任何封装头
                 （对应固件 esp_websocket_client_send_bin）
  5) 结束：      {"type":"stop"}  然后  {"type":"cancel"}

音频来源：用 ffmpeg 合成一段中文 TTS 不方便，改为生成一段「带话音特征的可辨识音」
          （440/880Hz 交替 + 白噪声包络，模拟说话能量起伏）。若云端只做 VAD/连通性
          校验，这足以让它「认为有人在说话」并触发返回；若云端要真识别语义，
          请改用 --wav 传入一段你自己的 16k 单声道人声 wav（脚本会用 ffmpeg 转 opus）。

依赖：ffmpeg（已确认在 PATH）、Python websockets（已装）。不需要 opuslib。

用法：
  python tools/ws_cloud_probe.py --token "<accessToken JWT>"
  python tools/ws_cloud_probe.py --token "<...>" --wav myvoice.wav
  python tools/ws_cloud_probe.py --token "<...>" --url wss://ai.strailine-space.com/ws/omni

退出即打印一份「诊断结论」，直接告诉你问题在云端还是固件。
"""

import argparse
import asyncio
import base64
import io
import json
import struct
import subprocess
import sys
import tempfile
import time
import os

# Windows 控制台默认 GBK，强制 stdout/stderr 用 UTF-8，避免 emoji/中文崩溃
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

try:
    import websockets
except ImportError:
    print("缺少 websockets：请先 python -m pip install websockets", file=sys.stderr)
    sys.exit(2)

DEFAULT_URL = "wss://ai.strailine-space.com/ws/omni"
SAMPLE_RATE = 16000
FRAME_MS = 60  # 固件帧时长 60ms


# ──────────────────────────────────────────────────────────────────────────
# JWT 解码：打印 token 的 iat/exp，先确认 token 没过期（省得白测）
# ──────────────────────────────────────────────────────────────────────────
def decode_jwt(token: str):
    try:
        payload_b64 = token.split(".")[1]
        payload_b64 += "=" * (-len(payload_b64) % 4)  # 补 padding
        payload = json.loads(base64.urlsafe_b64decode(payload_b64))
        return payload
    except Exception as e:
        return {"_decode_error": str(e)}


def check_token(token: str):
    p = decode_jwt(token)
    now = int(time.time())
    print("── Token 检查 ─────────────────────────────")
    if "_decode_error" in p:
        print(f"  ⚠ 无法解码 JWT：{p['_decode_error']}（可能不是标准 JWT，继续尝试）")
        return
    iat = p.get("iat")
    exp = p.get("exp")
    print(f"  sub(用户): {p.get('sub')}  phone: {p.get('phone')}")
    if iat:
        print(f"  签发 iat : {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(iat))}")
    if exp:
        print(f"  过期 exp : {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(exp))}")
        remain = exp - now
        if remain <= 0:
            print(f"  ❌ Token 已过期 {abs(remain)} 秒！请先在真机拿一个新 token 再测。")
        else:
            print(f"  ✅ Token 有效，剩余 {remain} 秒（{remain // 60} 分钟）")
    print()


# ──────────────────────────────────────────────────────────────────────────
# 用 ffmpeg 生成/转换出 raw OPUS 帧列表（每帧 60ms，16kHz 单声道）
#   ffmpeg 输出 ogg-opus，再用下面的 ogg 解析器切出逐帧 opus packet
# ──────────────────────────────────────────────────────────────────────────
def ffmpeg_to_ogg_opus(src_args, out_path):
    """src_args: ffmpeg 输入部分的参数列表。输出 16k 单声道 60ms 帧的 ogg-opus。"""
    cmd = [
        "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
        *src_args,
        "-ac", "1", "-ar", str(SAMPLE_RATE),
        "-c:a", "libopus", "-b:a", "24000",
        "-frame_duration", str(FRAME_MS),
        "-application", "voip",
        "-f", "ogg", out_path,
    ]
    subprocess.run(cmd, check=True)


def make_synth_ogg(out_path, seconds=4.0):
    """合成一段模拟话音：440/880Hz 交替 + 幅度起伏，让云端 VAD 判定为有人在说话。"""
    # lavfi 合成：两个正弦叠加，音量做 3Hz 起伏（模拟音节），限制到 4 秒
    fexpr = (
        "aevalsrc="
        "'0.4*sin(2*PI*440*t)*(0.6+0.4*sin(2*PI*3*t)) + "
        "0.3*sin(2*PI*880*t)*(0.5+0.5*sin(2*PI*2*t))'"
        f":s={SAMPLE_RATE}:d={seconds}"
    )
    ffmpeg_to_ogg_opus(["-f", "lavfi", "-i", fexpr], out_path)


def wav_to_ogg(wav_path, out_path):
    ffmpeg_to_ogg_opus(["-i", wav_path], out_path)


# ──────────────────────────────────────────────────────────────────────────
# 极简 Ogg 容器解析：从 ogg-opus 里抽出「逐个 opus packet」（含 60ms 音频帧）
#   跳过前两个 header packet（OpusHead / OpusTags）
# ──────────────────────────────────────────────────────────────────────────
def parse_ogg_opus_packets(data: bytes):
    packets = []
    pos = 0
    partial = b""
    n = len(data)
    while pos < n:
        if data[pos:pos + 4] != b"OggS":
            # 找下一个页同步
            nxt = data.find(b"OggS", pos + 1)
            if nxt < 0:
                break
            pos = nxt
            continue
        # Ogg page header
        # 0:  'OggS'
        # 5:  header_type
        # 26: page_segments (count)
        header_type = data[pos + 5]
        seg_count = data[pos + 26]
        seg_table = data[pos + 27: pos + 27 + seg_count]
        body_start = pos + 27 + seg_count
        # 依据 segment table 切包：值 <255 表示一个包在此结束
        idx = body_start
        cur = partial
        for lace in seg_table:
            cur += data[idx: idx + lace]
            idx += lace
            if lace < 255:
                packets.append(cur)
                cur = b""
        partial = cur  # 跨页续接的残包
        pos = idx
    # 前两个是 OpusHead / OpusTags，丢弃
    audio_packets = packets[2:] if len(packets) >= 2 else packets
    # 再滤掉空包
    return [p for p in audio_packets if p]


# ──────────────────────────────────────────────────────────────────────────
# 主流程：连接 → 握手 → 逐帧发 OPUS → 收全部回包 → 结束 → 诊断
# ──────────────────────────────────────────────────────────────────────────
async def run_probe(url, token, opus_frames, device_id, send_stop=False):
    full_url = f"{url}?token={token}"
    headers = {
        "Device-Id": device_id,
        "Client-Id": device_id,
        "Authorization": f"Bearer {token}",
        "Protocol-Version": "1",
    }

    print("── 连接云端 ───────────────────────────────")
    print(f"  URL     : {url}?token=<{len(token)}字符JWT>")
    print(f"  headers : Device-Id={device_id}  Authorization=Bearer <token>  Protocol-Version=1")
    print(f"  待发帧数: {len(opus_frames)} 帧 OPUS（每帧 {FRAME_MS}ms，共 ~{len(opus_frames)*FRAME_MS/1000:.1f}s 音频）")
    print()

    received = []          # 云端所有回包
    got_business = {"hit": False}   # 是否收到 started 之外的业务事件

    try:
        # websockets 库：additional_headers（新版）/ extra_headers（旧版）兼容
        connect_kwargs = dict(max_size=None, open_timeout=10, close_timeout=5)
        try:
            ws = await websockets.connect(full_url, additional_headers=headers, **connect_kwargs)
        except TypeError:
            ws = await websockets.connect(full_url, extra_headers=headers, **connect_kwargs)
    except Exception as e:
        print(f"❌ 连接失败：{type(e).__name__}: {e}")
        print("\n【诊断】连不上云端 —— 检查网络 / URL / token 是否被服务端拒绝握手。")
        return

    print("✅ WebSocket 已连接（TCP+TLS+握手成功）")

    async def reader():
        try:
            async for msg in ws:
                if isinstance(msg, bytes):
                    received.append(("bin", len(msg)))
                    print(f"  ← [二进制帧] {len(msg)} 字节（可能是 TTS 音频）")
                    got_business["hit"] = True
                else:
                    received.append(("text", msg))
                    print(f"  ← [文本帧] {msg}")
                    try:
                        t = json.loads(msg).get("type", "")
                    except Exception:
                        t = ""
                    # started/cancelled/error 都是协议控制消息，不算「识别结果」业务事件
                    if t not in ("started", "cancelled", "error", ""):
                        got_business["hit"] = True
        except websockets.ConnectionClosed:
            pass
        except Exception as e:
            print(f"  reader 异常：{type(e).__name__}: {e}")

    reader_task = asyncio.create_task(reader())

    # ① 发握手 start
    start_msg = json.dumps({
        "type": "start", "format": "opus",
        "sampleRate": SAMPLE_RATE, "deviceId": device_id,
    })
    await ws.send(start_msg)
    print(f"  → [文本帧] {start_msg}")

    # 等 started（最多 5 秒）
    t0 = time.time()
    while time.time() - t0 < 5 and not any(
        m[0] == "text" and '"started"' in m[1] for m in received
    ):
        await asyncio.sleep(0.05)

    # ② 逐帧发 OPUS（按 60ms 实时节奏，模拟真机）
    print(f"  → 开始逐帧发送 {len(opus_frames)} 帧 OPUS ...")
    for i, frame in enumerate(opus_frames):
        await ws.send(frame)  # 纯二进制，对应固件 send_bin
        if (i + 1) % 50 == 0:
            print(f"    已发送 {i+1} 帧")
        await asyncio.sleep(FRAME_MS / 1000.0)
    print(f"  → 音频发送完毕，共 {len(opus_frames)} 帧")

    # ③ 【严格复刻固件】固件对话流程只发 start + OPUS + cancel，从不发 stop
    #    （protocol_send_stop_listening 定义了但全项目无人调用）。
    #    默认不发 stop；仅当 --send-stop 时才补发，用于单独探测云端认不认 stop。
    if send_stop:
        await ws.send(json.dumps({"type": "stop"}))
        print('  → [文本帧] {"type":"stop"}  （注意：固件实际不发此信令，仅探测用）')

    # ④ 再等 15 秒，看云端会不会回识别结果
    print("  ⏳ 等待云端返回识别结果（最多 15 秒）...")
    t0 = time.time()
    while time.time() - t0 < 15 and not got_business["hit"]:
        await asyncio.sleep(0.1)

    # ⑤ 发 cancel 收尾并关闭
    try:
        await ws.send(json.dumps({"type": "cancel"}))
        print('  → [文本帧] {"type":"cancel"}')
        await asyncio.sleep(1.0)
        await ws.close()
    except Exception:
        pass
    reader_task.cancel()

    # ── 诊断结论 ──────────────────────────────────────────────
    print("\n══════════════ 诊断结论 ══════════════")
    texts = [m[1] for m in received if m[0] == "text"]
    bins = [m for m in received if m[0] == "bin"]
    print(f"  云端回包总数：文本 {len(texts)} 条，二进制 {len(bins)} 帧")
    if got_business["hit"]:
        print("  ✅ 云端返回了业务事件（识别结果 / TTS）！")
        print("     → 说明【云端本身正常】。真机发不过去是【固件↔云端之间的连接/网络】问题：")
        print("        重点查真机 WebSocket 是否真把二进制帧写出去了（对比服务端接收日志）。")
    else:
        errs = [t for t in texts if '"error"' in t]
        print("  ❌ 云端只回了控制消息（started/cancelled/error），没有任何识别结果 —— 与真机现象一致！")
        if errs:
            print(f"     ⚠ 云端还回了 error：{errs}")
            print("        → 说明固件发的某个 type 云端不认识，【固件与云端的协议契约对不上】。")
        print("     → 这证明【问题在云端侧或协议契约】：PC 直连、同样的 token、真 OPUS 音频，")
        print("        云端收到了音频却不返回识别结果。请拿本次时间戳去找后端确认：")
        print("        1) 服务端到底收没收到我这 67 帧音频？")
        print("        2) 云端期望的完整消息序列是什么？start 之后要不要发别的信令才触发 ASR？")
        print("        3) 云端认识哪些 type？（stop 报了 Unknown type）")
    print("═════════════════════════════════════")


def main():
    ap = argparse.ArgumentParser(description="Echo2 云端连通性探测（复刻固件 WS 契约）")
    ap.add_argument("--token", required=True, help="设备当前 accessToken（JWT），从真机日志 URI 的 ?token= 后面复制")
    ap.add_argument("--url", default=DEFAULT_URL, help=f"WebSocket 基础地址（默认 {DEFAULT_URL}）")
    ap.add_argument("--wav", default=None, help="可选：用你自己的人声 wav 测语义识别（脚本会用 ffmpeg 转 16k 单声道 opus）")
    ap.add_argument("--device-id", default="AE:4B:60:00:00:00", help="Device-Id（默认用真机 MAC 风格占位）")
    ap.add_argument("--seconds", type=float, default=4.0, help="合成音时长（不传 --wav 时生效）")
    ap.add_argument("--send-stop", action="store_true", help="额外补发 {\"type\":\"stop\"}（固件实际不发，仅用于探测云端认不认 stop）")
    args = ap.parse_args()

    check_token(args.token)

    # 生成 ogg-opus 到临时文件
    tmp = tempfile.NamedTemporaryFile(suffix=".ogg", delete=False)
    tmp.close()
    try:
        if args.wav:
            print(f"用 ffmpeg 转换 {args.wav} → 16k 单声道 OPUS ...")
            wav_to_ogg(args.wav, tmp.name)
        else:
            print(f"用 ffmpeg 合成 {args.seconds}s 模拟话音 → OPUS ...")
            make_synth_ogg(tmp.name, args.seconds)

        with open(tmp.name, "rb") as f:
            ogg = f.read()
        frames = parse_ogg_opus_packets(ogg)
        if not frames:
            print("❌ 从 ffmpeg 输出里没解析出 OPUS 帧，请检查 ffmpeg 是否正常。")
            sys.exit(1)
        print(f"✅ 已准备 {len(frames)} 帧 OPUS（帧大小示例：{[len(x) for x in frames[:5]]} ...）\n")
    finally:
        pass

    try:
        asyncio.run(run_probe(args.url, args.token, frames, args.device_id, args.send_stop))
    finally:
        try:
            os.unlink(tmp.name)
        except Exception:
            pass


if __name__ == "__main__":
    main()
