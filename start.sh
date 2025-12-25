cd src
poetry run uvicorn ui.backend.main:app --reload --port 5000 &
cd ..

cd src/ui/frontend
ionic serve --devapp-path src/ui/frontend --external
cd ../../..
