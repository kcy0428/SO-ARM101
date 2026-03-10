"""
쓰레기 감지 데이터셋 다운로드 스크립트
Roboflow의 공개 Garbage Detection 데이터셋 사용
"""

import os
import sys
from pathlib import Path

# 프로젝트 루트 기준 dataset 폴더
DATASET_DIR = Path(__file__).parent.parent / "dataset"
DATASET_DIR.mkdir(exist_ok=True)


def download_roboflow_dataset(api_key: str):
    """
    Roboflow에서 쓰레기 감지 데이터셋 다운로드

    사용 데이터셋: Garbage Detection Dataset
    https://universe.roboflow.com/material-identification/garbage-classification-3

    Args:
        api_key: Roboflow API 키 (https://roboflow.com 에서 무료 발급)
    """
    try:
        from roboflow import Roboflow
    except ImportError:
        print("[ERROR] roboflow 패키지가 없습니다. 설치 후 재시도하세요.")
        print("  pip install roboflow")
        sys.exit(1)

    print("[INFO] Roboflow 데이터셋 다운로드 시작...")

    rf = Roboflow(api_key=api_key)
    project = rf.workspace("material-identification").project("garbage-classification-3")
    dataset = project.version(1).download("yolov8", location=str(DATASET_DIR))

    print(f"[INFO] 다운로드 완료: {DATASET_DIR}")
    return dataset


def check_dataset():
    """데이터셋 구조 확인"""
    expected = ["train/images", "valid/images", "test/images", "data.yaml"]
    print("\n[INFO] 데이터셋 구조 확인:")
    for item in expected:
        path = DATASET_DIR / item
        status = "✅" if path.exists() else "❌"
        print(f"  {status} {path}")


if __name__ == "__main__":
    print("=" * 50)
    print("쓰레기 감지 데이터셋 다운로드")
    print("=" * 50)
    print()
    print("Roboflow API 키가 필요합니다.")
    print("1. https://roboflow.com 회원가입 (무료)")
    print("2. Settings > API Keys 에서 키 복사")
    print()

    api_key = input("API 키 입력: ").strip()
    if not api_key:
        print("[ERROR] API 키를 입력해주세요.")
        sys.exit(1)

    download_roboflow_dataset(api_key)
    check_dataset()
