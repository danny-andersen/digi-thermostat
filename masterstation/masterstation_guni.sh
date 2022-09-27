gunicorn -w 4 -b 0.0.0.0:5000 --log-level debug --log-file - --chdir /home/danny/FlaskApps --access-logfile - 'masterstation:app'

