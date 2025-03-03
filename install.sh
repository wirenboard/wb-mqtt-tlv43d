#!/bin/bash

# Обновляем список пакетов (только обновление списка, без upgrade)
echo "Обновление списка пакетов..."
sudo apt update -y

# Устанавливаем pip3, если он не установлен
if ! command -v pip3 &> /dev/null; then
    echo "Установка pip3..."
    sudo apt install -y python3-pip
else
    echo "pip3 уже установлен!"
fi

# Устанавливаем smbus2 через pip3
echo "Установка smbus2..."
pip3 install --user smbus2

# Проверяем успешность установки
python3 -c "import smbus2" &> /dev/null && echo "smbus2 успешно установлен!" || echo "Ошибка: smbus2 не удалось установить!"


SERVICE_NAME=tlv493.service
SERVICE_PATH=/etc/systemd/system/$SERVICE_NAME
SCRIPT_PATH=$(realpath $(dirname "$0")/tlv493.py)

# Создаём файл сервиса
cat <<EOF > $SERVICE_PATH
[Unit]
Description=Magnetometer Data Publisher
After=network.target

[Service]
ExecStart=/usr/bin/python3 $SCRIPT_PATH
Restart=always
User=root

[Install]
WantedBy=multi-user.target
EOF

# Обновляем systemd и включаем сервис
systemctl daemon-reload
systemctl enable $SERVICE_NAME
systemctl start $SERVICE_NAME

# Проверяем статус сервиса
systemctl status $SERVICE_NAME --no-pager
