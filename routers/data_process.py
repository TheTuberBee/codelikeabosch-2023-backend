from fastapi import APIRouter,  UploadFile
from common import parse_csv

router = APIRouter()

@router.post("/file")
async def fileUpload(file: UploadFile):
    try:
        if file.content_type != "text/csv":
            return {"error": "File must be in CSV format."}

        file_contents = await file.read()
        parsed_data = parse_csv(file_contents)
        
        if parsed_data:
            return {"data": parsed_data}
        else:
            return {"error": "Unable to parse CSV file."}
    except Exception as e:
        return {"error": f"An error occurred: {str(e)}"}

