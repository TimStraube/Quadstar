
poetry run uvicorn src.ui.backend.main:app --reload &

cd src/ui/frontend
ionic serve --devapp-path src/ui/frontend --external
cd ../../..
