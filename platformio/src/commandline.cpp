#include <Arduino.h>

//max 255
#define CMD_BUFFER_SIZE 10


String commandBuffer[CMD_BUFFER_SIZE];
uint8_t availableCommands = 0;

/// @brief Get a chunk of commands from the command line and put it in the command buffer.
void getCommandChunk(){
    char currChar;
    while (Serial.available() > 0){
        currChar = Serial.read();
        if (currChar == '\r'){
            //ignore CR
        }else if (currChar == '\n'){
            // advance to next command
            availableCommands++;
            Serial.println();//to add newline after echo print
        }else{
            // add char to current command
            commandBuffer[availableCommands].concat(currChar);
            Serial.print(currChar); //echo typed command
        }
    }
}

/// @brief Process all complete commands currently available.
void doCommands(){
    uint8_t processedCommmands = 0;

    while (processedCommmands < availableCommands) {
        if (strcmp(commandBuffer[processedCommmands].c_str(), "hello") == 0){
            Serial.println("hi!");
        }else if (strcmp(commandBuffer[processedCommmands].c_str(), "hi") == 0){
            Serial.println("hello!");
        }else{ //default
                Serial.print("Unknown command: ");
                Serial.println(commandBuffer[processedCommmands].c_str());
        }

        commandBuffer[processedCommmands] = ""; // clear command slot so we can append new stuff later
        processedCommmands++;
    }
    availableCommands = 0;
}