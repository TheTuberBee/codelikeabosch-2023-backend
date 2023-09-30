# Code #LikeABosch 2023 - Backend

Backend repository of our project, see the bottom of the page for the frontend.
You can try our project without installing, by clicking on this link:

https://codelikeabosch-2023-frontend.vercel.app/

If you are interested in our core logic, you can find it in `common/world.py`.

## Local development

Build container:
```
docker build -t bosch .
```

Run container with auto-reload:
```
docker run -p 127.0.0.1:8000:8000 -v .:/app bosch
```

Access the Swagger UI docs: http://localhost:8000/docs/

Jenkins deployment: https://dene.sh/bosch/api/

## Frontend repository

SvelteKit project which provides an awesome 3D view on the data generated by our API.
Upload a CSV dataset in the top bar, then start the simulation, and pay special attention to
the right-bottom events panel.
Note that while the app recieves the full demo before playing, there is nothing in a tick
that depends on any future ones.

https://github.com/vargaking/codelikeabosch-2023-frontend
