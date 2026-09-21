FROM python:3.10-slim

RUN apt update && apt install -y --no-install-recommends \
    python3-tk && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

COPY src/ /app/
WORKDIR /app
CMD ["python3", "detect_on_map.py"]
