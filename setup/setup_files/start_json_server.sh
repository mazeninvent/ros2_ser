# Load nvm into bash

source "$NVM_DIR/nvm.sh"

# Install and use Node.js 18 via nvm

# Dont use sudo for nvm-associated commands (npm, ng) since nvm is not accessible by root

nvm install 18

nvm use 18

cd json-server/

npm install
npm audit fix

#Stop if still running pib_api service
systemctl stop pib_api_boot.service

#Start the JOSN-Server
node server/json-server-funktion.mjs
