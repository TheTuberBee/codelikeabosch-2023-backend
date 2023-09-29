from fastapi import APIRouter

from . import file
from . import records


router = APIRouter()

router.include_router(file.router)
router.include_router(records.router)
