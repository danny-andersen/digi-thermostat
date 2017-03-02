if __name__ == "__main__":
    from bs4 import BeautifulSoup
    from datetime import datetime,time
    import sys

    if len(sys.argv) != 2:
	 sys.stderr.write("Please provide a file to parse\n")
	 sys.exit(1)
    html = BeautifulSoup(open(sys.argv[1]).read(), "lxml")
    def is_tr_with_timeclass(tag):
    	return tag.has_attr("class") and not tag.has_attr("id")
    res = ""
    hours = []
    for tr in html.find_all(class_="time"):
        for th in tr.find_all("th"):
	    for hr in th.find_all("span",class_="hour"):
		hours.append(hr.string)
    #print hours
    weather = []
    for tr in html.find_all(class_="weather-type"):
        for td in tr.find_all("td"):
	    for hr in td.find_all("img"):
	    	weather.append(hr["title"])
		#res = res + "\n" + td.string
    #print weather
    temp = []
    for tr in html.find_all(class_="temperature"):
	i = 0
        for td in tr.find_all("td"):
	    for hr in td.find_all("span",attrs={"data-unit": "c"}):
		i = True
	    	for st in hr.strings:
		    if (i):
		    	temp.append(st);
		    i = not i
		    #res = res + "\n" + td.string
    sunrise = html.find(class_="sunrise")
    sunriseStr = sunrise.string;
    sunriseHr = int(sunriseStr.split()[1].split(':')[0])
    sunset = html.find(class_="sunset")
    sunsetStr = sunset.string;
    sunsetHr = int(sunsetStr.split()[1].split(':')[0])

    #Find when the weather changes and say when
    startWeather = weather[0]
    end = 0
    endWeather = ""
    for s in weather:
	if s != startWeather:
	   endWeather = s
	   break
	end += 1
    if end >= len(hours): 
	end -= 1 
    forecast = startWeather + " until " + hours[end] + "00"
    if endWeather != "":
	forecast += " and then " + endWeather
    now = datetime.now().time() 
    if now.hour <= sunriseHr + 1:
	forecast = sunriseStr + ", " + forecast
    elif now.hour > 14 and now.hour <= sunsetHr + 1:
	forecast = sunsetStr + ", " + forecast
    print forecast
    expiry = abs(now.hour - int(hours[end])) * 3600 * 1000;
    #Save first hour temp as ext temp
    with open("setExtTemp.txt", "w") as f:
	f.truncate()
	f.write(temp[0] + "\n")
    with open("motd.txt", 'w') as f:
	f.truncate()
	f.write(forecast + "\n")
	f.write("%d" % expiry + "\n")

    
