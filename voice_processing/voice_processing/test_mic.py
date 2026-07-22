import pyaudio
p = pyaudio.PyAudio()

print("--- 사용 가능한 입력 장치 목록 ---")
for i in range(p.get_device_count()):
    dev = p.get_device_info_by_index(i)
    if dev.get('maxInputChannels') > 0:
        print(f"Index {i}: {dev.get('name')} (입력 채널: {dev.get('maxInputChannels')})")