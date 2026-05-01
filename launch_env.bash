#!/usr/bin/env bash
# ROS2 + venv 통합 환경 설정
# 사용법: source launch_env.bash

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GSAM_WS="${WS}"

source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash" 2>/dev/null || true

# venv site-packages (torch, transformers, supervision 등)
VENV_SITE="${WS}/gsam_ws_venv/lib/python3.12/site-packages"

# editable 설치 소스 경로 (pip install -e 는 .pth 파일로 등록되므로
# PYTHONPATH 방식에선 직접 명시해야 groundingdino/segment_anything 를 찾음)
GDINO_SRC="${WS}/external/GroundingDINO"
SAM_SRC="${WS}/external/segment-anything"

export PYTHONPATH="${VENV_SITE}:${GDINO_SRC}:${SAM_SRC}:${PYTHONPATH}"

# install/ 쪽 model_paths.yaml을 현재 워크스페이스 경로로 치환
# src/ yaml은 건드리지 않음 → git status 항상 clean 유지
_SRC_YAML="${WS}/src/grounded_sam_pkg/config/model_paths.yaml"
_INSTALL_YAML="${WS}/install/grounded_sam_pkg/share/grounded_sam_pkg/config/model_paths.yaml"

if [ -f "${_SRC_YAML}" ] && [ -f "${_INSTALL_YAML}" ]; then
    envsubst '$GSAM_WS' < "${_SRC_YAML}" > "${_INSTALL_YAML}"
fi

echo "[launch_env] ROS2 Jazzy + venv PYTHONPATH set"
echo "  ws       : ${GSAM_WS}"
echo "  venv     : ${VENV_SITE}"
echo "  gdino    : ${GDINO_SRC}"
echo "  sam      : ${SAM_SRC}"
