from fastapi import APIRouter, HTTPException

from models import Demo


router = APIRouter()


@router.get("/")
async def get_records():
    try:
        records = await Demo.all()
        return records

    except Exception as e:
        raise HTTPException(400, "Unable to retrieve records.")
