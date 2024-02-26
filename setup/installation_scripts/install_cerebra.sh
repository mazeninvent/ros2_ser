#!/bin/bash
#
# This script installs Cerebra
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Installing Cerebra --""$RESET_TEXT_COLOR"		

# Nginx variables
DEFAULT_NGINX_DIR="/etc/nginx"
DEFAULT_NGINX_HTML_DIR="$DEFAULT_NGINX_DIR/html"
NGINX_CONF_FILE="nginx.conf"

# Database variables
PHPLITEADMIN_ZIP="phpliteadmin_v1_9_9_dev.zip"
PHPLITEADMIN_INSTALLATION_DIR="/var/www/phpliteadmin"
DATABASE_DIR="$USER_HOME/pib_data"
DATABASE_FILE="pibdata.db"

# pib-api variables
PIB_API_DIR="$USER_HOME/flask"

# python code variables
PYTHON_CODE_PATH="$USER_HOME/cerebra_programs"
INIT_PYTHON_CODE="print('hello world')"

# Setup nginx
sudo apt install -y nginx
sudo mkdir -p $DEFAULT_NGINX_HTML_DIR

# Setting up nginx to serve Cerebra locally
sudo cp "$SETUP_FILES/nginx.conf" "$DEFAULT_NGINX_DIR/$NGINX_CONF_FILE"

# Install NVM (Node Version Manager)
NVM_DIR="/etc/nvm"
sudo mkdir "$NVM_DIR"
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/master/install.sh | bash

# Move nvm directory to systemfolders and create environment variable
sudo mv ~/.nvm "$NVM_DIR"
echo "export NVM_DIR=$NVM_DIR" | sudo tee -a /etc/profile.d/nvm.sh

# Source NVM script to make it available in the current shell
source "$NVM_DIR/nvm.sh"

# Install and use Node.js 19 via nvm
# Node.js version 18 causes a bug, getting all npm commands stuck loading
nvm install 19
nvm use 19

# Install Angular CLI
npm install -g @angular/cli

# Navigate to the folder where the Angular app is downloaded
cd $FRONTEND_DIR

# Install app dependencies and start build
sudo npm install
sudo ng build

# Undo directory change
cd $USER_HOME

# Move the build to the destination folder
sudo mv "$FRONTEND_DIR/dist"/* "$DEFAULT_NGINX_HTML_DIR"

# Install and configure phpLiteAdmin
sudo sed -i "s|;cgi.fix_pathinfo=1|cgi.fix_pathinfo=0|" /etc/php/8.1/fpm/php.ini
sudo mkdir "$PHPLITEADMIN_INSTALLATION_DIR"
sudo chown -R www-data:www-data "$PHPLITEADMIN_INSTALLATION_DIR"
sudo chmod -R 755 "$PHPLITEADMIN_INSTALLATION_DIR"
sudo unzip "$SETUP_FILES/$PHPLITEADMIN_ZIP" -d "$PHPLITEADMIN_INSTALLATION_DIR"
sudo systemctl restart php8.1-fpm

# Create the database (if it doesn't exist) and initialize it with the SQL file
mkdir "$DATABASE_DIR"
sudo chmod 777 "$USER_HOME"
sudo chmod 777 "$DATABASE_DIR"
sqlite3 "$DATABASE_DIR/$DATABASE_FILE" < "$SETUP_FILES/cerebra_init_database.sql"
sudo chmod 766 "$DATABASE_DIR/$DATABASE_FILE"
echo -e "$NEW_LINE""Database initialized successfully!"

# Create the directory for python code and populate it with a single initial python script (matching
# the single entry in the database)
mkdir "$PYTHON_CODE_PATH"
echo "$INIT_PYTHON_CODE" | cat > "$PYTHON_CODE_PATH/e1d46e2a-935e-4e2b-b2f9-0856af4257c5.py"

# Create pib-api
echo "export PYTHONIOENCODING=utf-8" >> $USER_HOME/.bashrc
pip3 install pipenv
cd $USER_HOME

pip install "$PIB_API_SETUP_DIR/client"
cp -r "$PIB_API_SETUP_DIR/flask" "$PIB_API_DIR"
sudo mv "$PIB_API_DIR/pib_api_boot.service" /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable pib_api_boot.service
cd $USER_HOME

# Open firefox without gui to generate default folder structures 
# This also avoids the welcome page the first time a user opens the browser
timeout 20s firefox --headless

# Set localhost as homepage in default profile
readonly FIREFOX_PREFS_FILE=$(echo /home/pib/snap/firefox/common/.mozilla/firefox/*.default)/prefs.js
echo "user_pref(\"browser.startup.homepage\", \"127.0.0.1\");" >> "$FIREFOX_PREFS_FILE"

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Cerebra installation completed --""$RESET_TEXT_COLOR""$NEW_LINE"