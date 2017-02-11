python -c 'if True:
    import sys, BeautifulSoup
    html = BeautifulSoup.BeautifulSoup(open(sys.argv[1]).read())
    def is_tr_with_bgcolor(tag):
    	return tag.has_attr('bgcolor') and not tag.has_attr('id')
    res = ""
    for tr in html.findAll("tr", bgcolor=True):
	i = 0
        for td in tr.findAll("td"):
	    i += 1
	    if (i == 2):
		print td.string
		#res = res + "\n" + td.string
    print res
' $1
