cd src
poetry run uvicorn ui.backend.main:app --reload --port 5001 &
cd ..

cd src/ui/frontend
ionic serve --devapp-path src/ui/frontend --external
cd ../../..
