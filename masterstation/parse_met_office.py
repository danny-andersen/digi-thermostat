if __name__ == "__main__":
    from datetime import datetime,time, timedelta
    import sys
    import xml.etree.ElementTree as ET

    if len(sys.argv) != 2:
	 sys.stderr.write("Please provide a file to parse\n")
	 sys.exit(1)

    weatherText = dict([(0,"Clear night"), (1,"Sunny day"), (2,"Partly cloudy"), \
		(3,"Partly cloudy"), (4,"Not used"), (5,"Mist"), (6,"Fog"), \
                (7,"Cloudy"), (8,"Overcast"), (9,"Light rain shower"), \
                (10,"Light rain shower"), (11,"Drizzle"), (12,"Light rain"), \
                (13,"Heavy rain shower"), (14,"Heavy rain shower"), \
                (15,"Heavy rain"), (16,"Sleet shower"), (17,"Sleet shower"), \
                (18,"Sleet"), (19,"Hail shower"), (20,"Hail shower"), \
                (21,"Hail"), (22,"Light snow shower"), (23,"Light snow shower"), \
                (24,"Light snow"), (25,"Heavy snow shower"), (26,"Heavy snow shower"),\
                (27,"Heavy snow"), (28,"Thunder shower"), (29,"Thunder shower"),\
                (30,"Thunder")])

    tree = ET.parse(sys.argv[1])
    root = tree.getroot()

    #Find start of hourly forecast
    now = datetime.today()
    today = now.strftime("%Y-%m-%dZ")
    tomorrow = (now + timedelta(days=1)).strftime("%Y-%m-%dZ")
    t = now.time()
    nowMins = (t.hour * 60) + t.minute
    sepMins = 24*60
    forecast = None
    temp = 0
    rainTime = -1
    days = root.iter("Period")
    for day in days:
        if day.get('value') == today:
            for rep in day.findall('Rep'):
                sep = abs(nowMins - int(rep.text))
		if sep < sepMins:
                    forecast = rep
                    sepMins = sep
        if day.get('value') == tomorrow:
	    mins = 24*60 - nowMins
            for rep in day.findall('Rep'):
                sep = mins + int(rep.text)
		if sep < sepMins:
                    forecast = rep
                    sepMins = sep
    nowWeather = int(forecast.get('W'))
    #Get next weather
    nextForecast = None
    nextWeather = -1
    nextTemp = -255
    precipTime = None
    stopRaining = None
    days = root.iter("Period")
    for day in days:
        if day.get('value') == today:
            for rep in day.findall('Rep'):
                if int(rep.text) > int(forecast.text):
                    if nextTemp == -255:
                        nextTemp = int(rep.get('T'))
		    if precipTime == None and int(rep.get('Pp')) > 50: # Rain more likely than not
			precipTime = rep
		    if stopRaining == None and int(rep.get('Pp')) <= 50: # Rain stopping
			stopRaining = rep
                    if weatherText[int(rep.get('W'))] != weatherText[nowWeather]:
		        nextForecast = rep
		        nextWeather = int(rep.get('W'))
        if day.get('value') == tomorrow:
	    mins = 24*60 - nowMins
            for rep in day.findall('Rep'):
		if precipTime == None and int(rep.get('Pp')) > 50: # Rain more likely than not
		    precipTime = rep
		if stopRaining == None and int(rep.get('Pp')) <= 50: # Rain stopping
		    stopRaining = rep
                if nextTemp == -255:
                    nextTemp = int(rep.get('T'))
                if nextWeather == -1 and weatherText[int(rep.get('W'))] != weatherText[nowWeather]:
	             nextForecast = rep
		     nextWeather = int(rep.get('W'))
        #print nextWeather, day.get('value'), today, tomorrow
    temp = (int(forecast.get('T')) + nextTemp) / 2.0
    wind = "%s-%smph" % (forecast.get('S'), forecast.get('G'))
    dirn = forecast.get('D')
    if len(dirn) == 3:
	wind += dirn
    else:
	wind += " %s" % dirn 
    #print forecast.text, nextForecast.text
    #print forecast.get('T'), nextTemp, temp, wind 
    #print nowWeather, nextWeather
    #print weatherText[nowWeather], weatherText[nextWeather]
    #print precipTime.text, precipTime.get('Pp')
    #print stopRaining.text, stopRaining.get('Pp')
    nextTime = int(nextForecast.text) / 60
    rainProb = int(forecast.get('Pp'))
    rainStr = ''
    if rainProb > 40: rainStr = " (Rain %s%%)" % rainProb 
    forecastText = "%s%s until %0d00" % (weatherText[nowWeather],rainStr,nextTime) 
    if rainProb <= 50 and precipTime != None: #Currently not raining (probably)
        rainTime = int(precipTime.text) / 60
	forecastText += ". %s (%s%%) at %0d00" % (weatherText[precipTime.get('W')], precipTime.get('Pp'),rainTime)
    else:
        rainProb = int(nextForecast.get('Pp'))
        rainStr = ''
        if rainProb > 40: rainStr = " (Rain %s%%)" % rainProb 
	forecastText += " and then %s%s" % (weatherText[nextWeather],rainStr)
    if now.hour > nextTime:
        expiry = (24 - now.hour + nextTime) * 3600 * 1000
    else:
        expiry = nextTime - now.hour * 3600 * 1000

    #print forecastText, expiry
    #print wind, temp

    #Save output into files for sending to thermostat
    #Save first hour temp as ext temp
    with open("setExtTemp.txt", "w") as f:
	f.truncate()
	f.write(str(temp) + "\n")
    	f.write(wind + "\n")
    #Add expiry to motd
    with open("motd.txt", 'w') as f:
	f.truncate()
	f.write(forecastText + "\n")
	f.write("%d" % expiry + "\n")
    
