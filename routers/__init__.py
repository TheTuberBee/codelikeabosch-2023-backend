from fastapi import APIRouter

from . import demo


router = APIRouter()

router.include_router(demo.router, prefix="/demo")
