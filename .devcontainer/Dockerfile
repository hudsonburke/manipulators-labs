FROM mcr.microsoft.com/vscode/devcontainers/python:3.12

# MediaPipe loads EGL/GLES lazily when the browser's first image is processed.
# OpenCV still needs its shared libraries even without native GUI windows.
RUN apt-get update \
    && apt-get install -y --no-install-recommends libegl1 libgles2 libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install uv without requiring students to configure Python separately.
COPY --from=ghcr.io/astral-sh/uv:0.13.9 /uv /uvx /usr/local/bin/
