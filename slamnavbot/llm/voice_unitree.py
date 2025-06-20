import sounddevice as sd
import subprocess
import time

from scipy.io.wavfile import write
from faster_whisper import WhisperModel
from openai import OpenAI
import re, os
import pandas as pd
import openai

openai.api_key = "你的OpenAI_API_KEY"

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class Speaker_unitree():
    def __init__(self, ip_g1: str = None):
        try:
            ChannelFactoryInitialize(0, ip_g1)
        except Exception as e:
            print("Exception init speaker:", e)

        self.audio_client = AudioClient()
        self.audio_client.SetTimeout(10.0)
        self.audio_client.Init()
        self.audio_client.SetVolume(85)

    def speak(self, speak_text):
        # green led
        self.audio_client.LedControl(0, 255, 0)
        self.audio_client.TtsMaker(speak_text, 0)
        time.sleep(len(speak_text) * 0.26)
        # blue led
        self.audio_client.LedControl(0, 0, 255)

    def get_volume(self):
        ret = self.audio_client.GetVolume()
        print("Volume: ", ret)
        return ret


def speak(text, voice='zh', speed=200):
    subprocess.run(["espeak", f"-v{voice}", f"-s{speed}", text])


speaker_unitree = Speaker_unitree(ip_g1="eno1")


# 初始化 Whisper 模型（可以改成 "medium"、"large-v2"）
model = WhisperModel("tiny", device="auto", compute_type="int8", cpu_threads=8)

# 候选位置
candidate_locations = ['冰箱', '电梯', '咖啡机', '洗手池', '垃圾桶', '机器人实验室']


def record_audio(filename="recording.wav", duration=3.5, fs=16000, ):
    print("🎤 开始录音，说话吧...")
    recording = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='int16')
    sd.wait()
    write(filename, fs, recording)
    print("✅ 录音完成")


def transcribe_audio(path):
    segments, info = model.transcribe(path, beam_size=5)
    text = "".join([seg.text for seg in segments])
    return text.strip()


def get_xyz_from_excel(location_name, excel_path='/home/zqh/yyx_G1/multiagent-isaacsim/llm/home.xlsx'):
    df = pd.read_excel(excel_path, header=None)  # 👈 不使用第一行作为列名
    row = df[df.iloc[:, 0] == location_name]  # 第一列是位置名
    if not row.empty:
        x = row.iloc[0, 1]  # 第二列 X
        y = row.iloc[0, 2]  # 第三列 Y
        z = row.iloc[0, 3]  # 第四列 Z
        return x, y, z
    else:
        return None, None, None


def extract_location(text, candidates):
    prompt = f"""你是一个导航助理。我会给你一句中文命令，请你从以下候选位置中判断这个命令最可能是在哪个位置执行的：
候选位置：{", ".join(candidates)}
命令：{text}
请你在[]中只返回一个位置，不要解释。"""

    # client = OpenAI(api_key="sk-4b2b473f8d8149109dea837a488fa273", base_url="https://api.deepseek.com")
    client = OpenAI(api_key="sk-MQ8Zb5grxtM4hniFfyAWeblLcDxwNKG9tP64PGrfAaAePbr7", base_url="https://vip.dmxapi.com/v1")

    response = client.chat.completions.create(
        model="deepseek-chat",
        # model="gpt-4o",
        messages=[
            {"role": "system", "content": "你是一个导航助理"},
            {"role": "user", "content": prompt},
        ],
        stream=False
    )
    return response.choices[0].message.content


def respond(text, location, ip_speaker: str = None):
    response = f"我将带你去{location}"
    print("🤖 回应：", response)
    if ip_speaker:
        speaker_unitree.speak(response)
    else:
        speak(response)


def extract_bracket_content(text):
    match = re.search(r'\[([^\[\]]+)\]', text)
    if match:
        return match.group(1)
    return None


def run_once(file_path: str = None, ip_speaker: str = None):
    if ip_speaker:
        speaker_unitree.speak("你好，我是导航机器人，你有什么需求")
    else:
        speak("你好，我是导航机器人，你有什么需求")
    if file_path is None:
        record_audio(duration=4)
        file_path = f"recording.wav"
    text = transcribe_audio(file_path)
    print("📝 你说的是：", text)
    if not text:
        print("❌ 没听清")
        return
    location = extract_location(text, candidate_locations)
    location = extract_bracket_content(location)
    print(location)
    respond(text, location, ip_speaker=ip_speaker)
    x, y, z = get_xyz_from_excel(location)
    print(f'x: {x}')
    print(f'y: {y}')
    print(f'z: {z}')
    return x, y, z
