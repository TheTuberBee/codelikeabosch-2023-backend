from fastapi import APIRouter
from . import data_process

router = APIRouter()

router.include_router(data_process.router)
