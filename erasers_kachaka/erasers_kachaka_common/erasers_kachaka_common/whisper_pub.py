#!/usr/bin/env python
# -*- coding: utf-8 -*-

import speech_recognition as sr
import sounddevice
import numpy as np
import soundfile as sf
import io
import tempfile
from faster_whisper import WhisperModel
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceRecog(Node):
    def __init__(self):
        super().__init__("whisper_node")
        # speech_recognition
        self.r = sr.Recognizer()
        self.m = sr.Microphone()

        # whisper
        self.model_size = "small"
        self.model = WhisperModel(self.model_size, device="cuda", compute_type="float32")

        #認識した言葉をためておく
        self.recognized_texts = []

        self.recog_pub = self.create_publisher(String, "/voice_recog",10)

        # 別スレッドで音声認識を実行
        #self.recognition_thread = threading.Thread(target=self.whisper_recog)
        #self.recognition_thread.daemon = True
        #self.recognition_thread.start()

    def whisper_recog(self):
        try:
            #マイクを起動
            with self.m as source:
                #雑音のしきい値の調整
                self.r.adjust_for_ambient_noise(source)

            while rclpy.ok():
                print("Say something!")
                with self.m as source:
                    #音声受取
                    audio = self.r.listen(source)
                    print("Got it! Now to recognize it...")

                    # WAVデータをBytesIOに変換する
                    wav_bytes = audio.get_wav_data()
                    wav_stream = io.BytesIO(wav_bytes)

                    # soundfileで読み込む
                    audio_array, sampling_rate = sf.read(wav_stream, dtype="float32")

                    # 一時ファイルに保存
                    with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as tmp_wav:
                        sf.write(tmp_wav.name, audio_array, sampling_rate)

                        # faster-whisperで音声認識
                        #segments, _ = model.transcribe(tmp_wav.name, language="ja")
                        segments, _ = self.model.transcribe(tmp_wav.name, language="en")

                        # 結果をリストに追加
                        for segment in segments:
                            self.recognized_texts.append(segment.text)
                            print(segment.text)

                        #トピックへ送信
                        msg = String()
                        msg.data = segment.text
                        self.recog_pub.publish(msg)

        except KeyboardInterrupt:
            print("Recognized texts:")
            print(self.recognized_texts)

def main():
    rclpy.init()
    node = VoiceRecog()
    try:
        #rclpy.spin(node)
        node.whisper_recog()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            pass

if __name__ == "__main__":
    main()
