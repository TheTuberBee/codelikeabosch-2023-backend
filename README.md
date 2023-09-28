# Code #LikeABosch 2023 - Backend

Build container:
```
docker build -t bosch .
```

Run container with auto-reload:
```
docker run -p 127.0.0.1:8000:8000 -v .:/app bosch
```

Access the Swagger UI docs: http://localhost:8000/docs

Deployed API available: https://dene.sh/bosch
