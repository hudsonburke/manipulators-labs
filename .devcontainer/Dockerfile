FROM mcr.microsoft.com/vscode/devcontainers/python:3.13

# Install uv without requiring students to configure Python separately.
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /usr/local/bin/
