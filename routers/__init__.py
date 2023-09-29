from fastapi import APIRouter
from . import data_process

router = APIRouter(prefix="/api")

router.include_router(data_process.router)
