#!/bin/bash

date >> /Motion/Motion_send.log
echo $1 >> /Motion/Motion_send.log

cd /srv/bot
#php -e viber_send.php $1
php -e telegram_send.php $1
