FROM python:3.11-alpine

# Keeps Python from generating .pyc files in the container
ENV PYTHONDONTWRITEBYTECODE=1

# Turns off buffering for easier container logging
ENV PYTHONUNBUFFERED=1

WORKDIR /app

COPY . /app

RUN pip install -r requirements.txt --no-cache-dir

EXPOSE 8000

CMD ["python", "main.py"]
