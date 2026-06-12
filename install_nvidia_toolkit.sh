#!/bin/bash
# install_nvidia_toolkit.sh

if [ -f /etc/arch-release ]; then
    echo "▶ Arch Linux detected."
    # 1. Sync database and install using pacman
    # Using -Syu to avoid partial upgrade issues which often cause 404s
    sudo pacman -Syu --needed --noconfirm nvidia-container-toolkit

elif [ -f /etc/debian_version ]; then
    echo "▶ Debian/Ubuntu detected."
    # 1. Add repository
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
      sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
      sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

    # 2. Install
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
else
    echo "❌ Unsupported OS. Please install nvidia-container-toolkit manually."
    exit 1
fi

# 3. Configure Docker runtime (common for both)
echo "▶ Configuring Docker runtime..."
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

echo "✔ Done. Please check with 'nvidia-smi' inside docker."
