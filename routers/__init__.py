from fastapi import APIRouter

from . import demo
from . import records


router = APIRouter()

router.include_router(demo.router, prefix="/demo")
router.include_router(records.router)
