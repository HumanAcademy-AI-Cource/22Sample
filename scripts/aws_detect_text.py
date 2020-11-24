#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ライブラリをインポート
import time
import random
import boto3


def detect_text(image_path):
    """
    AWSを使った画像中の文字を認識する関数
    """
    
    # 画像認識の準備
    rekognition = boto3.client(service_name="rekognition")
    with open(image_path, 'rb') as file:
        try:
            # 画像中から文字を認識
            detext_text_data = rekognition.detect_text(Image={'Bytes': file.read()})
            if len(detext_text_data["TextDetections"]) != 0:
                # 認識結果から文字だけを取り出す
                text = detext_text_data["TextDetections"][0]["DetectedText"]
                print("認識結果: {0}".format(text))
            else:
                text = ""
                print("画像中に文字が検出されませんでした。")
        except Exception as e:
            print("AWSが混み合っていますので、しばらくお待ちください。")
            text = ""
            time.sleep(int(random.uniform(0, 5)))
    return text

if __name__ == '__main__':
    # 画像のパス
    image_path = "../img/forward.jpg"
    # 画像中の文字を調べる
    text = detect_text(image_path)