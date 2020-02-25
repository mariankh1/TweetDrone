#This code creates the dataset from Corpus.csv which is downloadable from the
#internet well known dataset which is labeled manually by hand. But for the text
#of tweets you need to fetch them with their IDs.
import tweepy

# Twitter Developer keys here
# It is CENSORED
consumer_key = 'ZbkvAkuMm7qOjlDclZ4SYD2ET'
consumer_key_secret = 'QrFhVrbUnm0KGRb8j6dR4Zt7zyvcTrHbZMAMoUZ7utY3MS2YXx'
access_token = '207479429-fDdOtZkE53fkFPYcZo7aiXS0H6s0vo08mUlpIOaT'
access_token_secret = 'LgWGAOUJS9lyCoFFMhy9XEqLfxggjL9bbSyohpQ6jfSMn'

auth = tweepy.OAuthHandler(consumer_key, consumer_key_secret)
auth.set_access_token(access_token, access_token_secret)
api = tweepy.API(auth)

# This method creates the training set
def createTrainingSet(corpusFile, targetResultFile):
    import csv
    import time

    counter = 0
    corpus = []

    with open(corpusFile, 'r') as csvfile:
        lineReader = csv.reader(csvfile, delimiter=',', quotechar="\"")
        for row in lineReader:
            corpus.append({"tweet_id": row[0]})

    sleepTime = 2
    trainingDataSet = []

    for tweet in corpus:
        try:
            tweetFetched = api.get_status(tweet["tweet_id"])
            print("Tweet fetched" + tweetFetched.text)
            if "sandy" in tweetFetched.text:
             if tweetFetched.coordinates:
              print ('lat:', tweetFetched.coordinates["coordinates"][0])
              print ('lon:', tweetFetched.coordinates["coordinates"][1])
              tweet["coordinates"] =  tweetFetched.coordinates["coordinates"]
             tweet["text"] = tweetFetched.text
             tweet["lat"] = tweetFetched.coordinates["coordinates"][0]
             tweet["lon"] = tweetFetched.coordinates["coordinates"][1]
             trainingDataSet.append(tweet)
             time.sleep(sleepTime)

        except:
            print("Inside the exception - no:2")
            continue

    with open(targetResultFile, 'w') as csvfile:
        linewriter = csv.writer(csvfile, delimiter=',', quotechar="\"")
        for tweet in trainingDataSet:
            try:
                linewriter.writerow([tweet["tweet_id"], tweet["text"],tweet["lat"],tweet["lon"]])
            except Exception as e:
                print(e)
    return trainingDataSet

# Code starts here
# This is corpus dataset
corpusFile = "datasets/sandydataset/xab"
# This is my target file
targetResultFile = "datasets/sandyTrueSortedPart2.csv"
# Call the method
resultFile = createTrainingSet(corpusFile, targetResultFile)
