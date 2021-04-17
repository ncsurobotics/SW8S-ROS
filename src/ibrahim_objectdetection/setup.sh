cd objectClassification
mkdir weights
cd weights
wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1IgkL5iOM1cbNLBH9G2jsIL-rrPOc77uY' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1IgkL5iOM1cbNLBH9G2jsIL-rrPOc77uY" -O yolov3_training_final.weights && rm -rf /tmp/cookies.txt

