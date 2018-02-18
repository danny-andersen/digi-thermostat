import sys  
from PyQt4.QtGui import *  
from PyQt4.QtCore import *  
from PyQt4.QtWebKit import *  
from lxml import html 

class Render(QWebPage):  
  def __init__(self, url):  
    self.app = QApplication(sys.argv)  
    QWebPage.__init__(self)  
    self.loadFinished.connect(self._loadFinished)  
    self.mainFrame().load(QUrl(url))  
    self.app.exec_()  
  
  def _loadFinished(self, result):  
    self.frame = self.mainFrame()  
    self.app.quit() 

if __name__ == "__main__":
    from pyvirtualdisplay import Display
    #from selenium import webdriver
    from bs4 import BeautifulSoup
    from datetime import datetime,time
    import sys

    display = Display(visible=0, size=(1024, 768))
    display.start()

    url = "http://www.bbc.co.uk/weather/2642573"

    #browser = webdriver.Chrome('/usr/lib/chromium-browser/chromedriver')
    #browser.get(url)
    #resultHtml = browser.execute_script("return document.body.innerHTML")

    r = Render(url)  
    result = r.frame.toHtml()
    resultHtml = str(result.toAscii())
    #print resultHtml
    with open("bbc.html", "w") as f:
	f.truncate()
	f.write(resultHtml + "\n")
    print "Saved html"
    display.stop()

    print "Parsing Html"
    html = BeautifulSoup(resultHtml, "lxml")
    #Find start of hourly forecast
    hourly = html.find("table", class_="moving-window");
    hours = []
    for tr in hourly.find_all(class_="time"):
        for th in tr.find_all("th"):
	    for hr in th.find_all("span",class_="hour"):
		hours.append(hr.string)
    print hours
    weather = []
    for tr in hourly.find_all(class_="weather-type"):
        for td in tr.find_all("td"):
	    for hr in td.find_all("img"):
	    	weather.append(hr["title"])
    print weather
    temp = []
    for tr in hourly.find_all(class_="temperature"):
	i = 0
        for td in tr.find_all("td"):
	    for hr in td.find_all("span",attrs={"data-unit": "c"}):
		i = True
	    	for st in hr.strings:
		    if (i):
		    	temp.append(st);
		    i = not i
    #Use observation module
    #tempTag = html.find("p", class_="temperature");
    #if (tempTag != None):
    #    for hr in tempTag.find_all("span",attrs={"data-unit": "c"}):
    #	   for st in hr.strings:
    #	       temp.append(st);
    print temp
    windSpeed = []
    windDirection = []
    for tr in hourly.find_all(class_="windspeed"):
        for td in tr.find_all("td"):
	   for hr in td.find_all("span",class_="windspeed-value-unit-mph"):
	    	for st in hr.stripped_strings:
    	   	    windSpeed.append(st);
	   for hr in td.find_all("span",class_="description"):
	    	for st in hr.stripped_strings:
    	   	    windDirection.append(st);
    windDir = windDirection[0].split()
    windDirAbbr = ""
    for wdw in windDir:
	windDirAbbr += wdw[0];
    wind = windSpeed[0]+windSpeed[1]+" " + windDirAbbr

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
    #Look for rain
    rainTime = 0
    rain = ""
    for s in weather:
	if s == "Thunderstorm" or s.lower().find("rain") != -1:
	    rain = s
	    break
        rainTime += 1
    if rainTime >= len(hours): 
	rainTime = -1 

    #Put it all together
    forecast = startWeather + " until " + hours[end] + "00"
    if rainTime != -1 and rainTime != end and rain != startWeather:
	forecast += ". " + rain + " at " + hours[rainTime] + "00"
    elif endWeather != "":
	forecast += " and then " + endWeather
    now = datetime.now().time() 
    if now.hour <= sunriseHr + 1:
	forecast = sunriseStr + ", " + forecast
    elif now.hour > 14 and now.hour <= sunsetHr + 1:
	forecast = sunsetStr + ", " + forecast
    print forecast
    expiry = abs(now.hour - int(hours[end])) * 3600 * 1000;

    #Save output into files for sending to thermostat
    #Save first hour temp as ext temp
    with open("setExtTemp.txt", "w") as f:
	f.truncate()
	f.write(temp[0] + "\n")
    	f.write(wind + "\n")
    #Add expiry to motd
    with open("motd.txt", 'w') as f:
	f.truncate()
	f.write(forecast + "\n")
	f.write("%d" % expiry + "\n")
    
