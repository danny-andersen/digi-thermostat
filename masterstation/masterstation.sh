uwsgi --http 0.0.0.0:5000 --rem-header Content-type --master --workers 4 -w masterstation:app

