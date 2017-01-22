#!/bin/bash

[ -f passwords.cmake ] && rm passwords.cmake
echo "set(USERNAME ${DRISHTISDK_BOT_USER})" > passwords.cmake
echo "set(PASSWORD ${DRISHTISDK_BOT_PASS})" >> passwords.cmake
