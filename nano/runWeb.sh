cd ~/VapaaCruiser/nano/web/
gunicorn --bind=0.0.0.0:5000 web:app --daemon
