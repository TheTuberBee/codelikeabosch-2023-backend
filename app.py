from fastapi import FastAPI
import uvicorn
from routers import router
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import ORJSONResponse
from tortoise.contrib.fastapi import register_tortoise
import dotenv
import os

api = FastAPI(
     default_response_class=ORJSONResponse,
)

api.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)

dotenv.load_dotenv()

db_config: dict = {
    'connections': {
        # Dict format for connection
        'default': {
            'engine': 'tortoise.backends.asyncpg',
            'credentials': {
                'host': os.environ["DB_HOST"],
                'port': os.environ["DB_PORT"],
                'user': os.environ["DB_USER"],
                'password': os.environ["DB_PASSWORD"],
                'database': os.environ["DB_DATABASE"],
            },
        },
    },
    'apps': {
        'models': {
            'models': ['models'],
            # If no default_connection specified, defaults to 'default'
            'default_connection': 'default',
        }
    }
}

register_tortoise(
    app=api,
    config=db_config,
    generate_schemas=True
)

api.include_router(router, prefix="/api")

@api.get("/")
def root():
    return "Hello World!"

if __name__ == "__main__":
    uvicorn.run(api, host="0.0.0.0", port=8000)
