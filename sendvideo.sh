#!/bin/bash

date >> /Motion/Motion_send_video.log
echo $1 >> /Motion/Motion_send_video.log

cd /srv/bot
php -e telegram_send_video.php $1
