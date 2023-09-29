from fastapi import APIRouter

from . import file


router = APIRouter()

router.include_router(file.router)
