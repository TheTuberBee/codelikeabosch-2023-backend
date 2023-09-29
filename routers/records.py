from fastapi import APIRouter, HTTPException

from models import Playbook


router = APIRouter()


@router.get("/")
async def get_records():
    try:
        records = await Playbook.all()
        return records

    except Exception as e:
        raise HTTPException(400, "Unable to retrieve records.")
