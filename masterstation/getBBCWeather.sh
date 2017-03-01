#wget http://weather.yahooapis.com/forecastrss?w=28915&u=c
#wget -O yahooWeather.json https://query.yahooapis.com/v1/public/yql?q=select%20*%20from%20weather.forecast%20where%20woeid%20%3D%2028915&format=json&env=store%3A%2F%2Fdatatables.org%2Falltableswithkeys

//BBC
wget -O bbc-weather.html http://www.bbc.co.uk/weather/2642573
python parse_bbc.py bbc-weather.html

