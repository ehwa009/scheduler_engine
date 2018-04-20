'use strict';

/**
 * This sample demonstrates a simple skill built with the Amazon Alexa Skills Kit.
 * The Intent Schema, Custom Slots, and Sample Utterances for this skill, as well as
 * testing instructions are located at http://amzn.to/1LzFrj6
 *
 * For additional samples, visit the Alexa Skills Kit Getting Started guide at
 * http://amzn.to/1LGWsLG
 */


// --------------- Helpers that build all of the responses -----------------------

function buildSpeechletResponse(title, output, repromptText, shouldEndSession) {
    return {
        outputSpeech: {
            type: 'PlainText',
            text: output,
        },
        card: {
            type: 'Simple',
            title: `SessionSpeechlet - ${title}`,
            content: `SessionSpeechlet - ${output}`,
        },
        reprompt: {
            outputSpeech: {
                type: 'PlainText',
                text: repromptText,
            },
        },
        shouldEndSession,
    };
}


function buildResponse(sessionAttributes, speechletResponse) {
    return {
        version: '1.0',
        sessionAttributes,
        response: speechletResponse,
    };
}


function buildSpeechletResponseWithDirectiveNoIntent() {
    console.log("in buildSpeechletResponseWithDirectiveNoIntent");
    return {
      "outputSpeech" : null,
      "card" : null,
      "directives" : [ {
        "type" : "Dialog.Delegate"
      } ],
      "reprompt" : null,
      "shouldEndSession" : false
    }
  }


// --------------- Functions that control the skill's behavior -----------------------

function getWelcomeResponse(callback) {
    // If we wanted to initialize the session to have some attributes we could add those here.
    const sessionAttributes = {};
    const cardTitle = 'Welcome';
    const speechOutput = "Hello, how may I help you?";
    // If the user either does not reply to the welcome message or says something that is not
    // understood, they will be prompted again with this text.
    const repromptText = "What can I do for you?";
    const shouldEndSession = false;

    callback(sessionAttributes,
        buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
}


function handleSessionEndRequest(callback) {
    const cardTitle = 'Session Ended';
    const speechOutput = 'Thank you for using receptionist at Auckland University. ' +
                    'Have a nice day!';
    // Setting this to true ends the session and exits the skill.
    const shouldEndSession = true;

    callback({}, buildSpeechletResponse(cardTitle, speechOutput, null, shouldEndSession));
}

function createTargetNameAttribute(targetName) {
    return {
        targetName,
    };
}

function createUserNameAttribute(userName) {
    return {
        userName,
    };
}


function delegateSlotCollection(request, sessionAttributes, callback){
  console.log("in delegateSlotCollection");
  console.log("  current dialogState: "+JSON.stringify(request.dialogState));

    if (request.dialogState === "STARTED") {
      console.log("in started");
      console.log("  current request: "+JSON.stringify(request));
      var updatedIntent=request.intent;
      //optionally pre-fill slots: update the intent object with slot values for which
      //you have defaults, then return Dialog.Delegate with this updated intent
      // in the updatedIntent property
      callback(sessionAttributes,
          buildSpeechletResponseWithDirectiveNoIntent());
    } else if (request.dialogState !== "COMPLETED") {
      console.log("in not completed");
      console.log("  current request: "+JSON.stringify(request));
      // return a Dialog.Delegate directive with no updatedIntent property.
      callback(sessionAttributes,
          buildSpeechletResponseWithDirectiveNoIntent());
    } else {
      console.log("in completed");
      console.log("  current request: "+JSON.stringify(request));
      console.log("  returning: "+ JSON.stringify(request.intent));
      // Dialog is now complete and all required slots should be filled,
      // so call your normal intent handler.
      return request.intent;
    }
}


function setPeopleName(intentRequest, intent, session, callback) {
    const cardTitle = intent.name;
    const targetName = intent.slots.target_name;
    const userName = intent.slots.user_name;
    const time = intent.slots.time;

    let repromptText = '';
    let sessionAttributes = {};
    var filledSlots = delegateSlotCollection(intentRequest, sessionAttributes, callback);

    const shouldEndSession = false;
    let speechOutput = '';

    const targetPerson = targetName.value;
    const userPerson = userName.value;
    if (targetName || userName) {
        sessionAttributes = createTargetNameAttribute(targetPerson);
        sessionAttributes['userName'] = userPerson;

        speechOutput = `Hi ${userPerson}, I have checked you in. please have a seat, ${targetPerson} will be with you shortly`;
        repromptText = "Did he know you are coming?";
    
    } else {
        speechOutput = `I'm not sure + ${targetPerson} is in our building. Can you contact yourself?`;
        repromptText = null;
    }
    console.log(sessionAttributes)
    callback(sessionAttributes,
            buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
}

function getWaitingTime(intent, session, callback) {
    let targetPerson;
    let userPeron;

    const repromptText = null;
    const sessionAttributes = {};
    let shouldEndSession = true;
    let speechOutput = '';

    if (session.attributes) {
        targetPerson = session.attributes.targetName;
        userPeron = session.attributes.userName;
    }

    if (targetPerson || userPeron) {
        speechOutput = `Hi ${userPeron}, ${targetPerson} is current busy with a another patient, 
                        the estimated waiting time is 7 mins.`;
    } else {
        speechOutput = "";
    }
    callback(sessionAttributes,
         buildSpeechletResponse(intent.name, speechOutput, repromptText, shouldEndSession));
}

function getAppDuration(intent, session, callback) {
    const repromptText = null;
    const sessionAttributes = {};
    let shouldEndSession = true;
    let speechOutput = '';

    if (session.attributes) {
        speechOutput = `Our standard appointments with the Doctor are 15mins long, 
                        unless you have made prior arrangements`;
    } else {
        speechOutput = "";
    }
    callback(sessionAttributes,
        buildSpeechletResponse(intent.name, speechOutput, repromptText, shouldEndSession));
}


function getPersonLocation(intent, session, callback) {
    let intendedPerson;
    const repromptText = null;
    const sessionAttributes = {};
    let shouldEndSession = false;
    let speechOutput = '';

    if (session.attributes) {
        intendedPerson = session.attributes.intendedPerson;
    }

    if (intendedPerson) {
        speechOutput = `Good. You may find ${intendedPerson} straight down the corridor in the ECE staff offices.`;
    } else {
        speechOutput = "I don't know who are you looking for. You can say, I have meeting with Eddie."
    }
    callback(sessionAttributes,
         buildSpeechletResponse(intent.name, speechOutput, repromptText, shouldEndSession));
}


// --------------- Events -----------------------

/**
 * Called when the session starts.
 */
function onSessionStarted(sessionStartedRequest, session) {
    console.log(`onSessionStarted requestId=${sessionStartedRequest.requestId}, sessionId=${session.sessionId}`);
}

/**
 * Called when the user launches the skill without specifying what they want.
 */
function onLaunch(launchRequest, session, callback) {
    console.log(`onLaunch requestId=${launchRequest.requestId}, sessionId=${session.sessionId}`);

    // Dispatch to your skill's launch.
    getWelcomeResponse(callback);
}

/**
 * Called when the user specifies an intent for this skill.
 */
function onIntent(intentRequest, session, callback) {
    console.log(`onIntent requestId=${intentRequest.requestId}, sessionId=${session.sessionId}`);

    const intent = intentRequest.intent;
    const intentName = intentRequest.intent.name;

    // Dispatch to your skill's intent handlers
    if (intentName === 'receptionMeeting') {
        setPeopleName(intentRequest, intent, session, callback);
    } else if (intentName === 'waiting') {
        getWaitingTime(intent, session, callback);
    } else if (intentName === 'appointmentDuration') {
        getAppDuration(intent, session, callback);
    } else if (intentName === 'receptionPersonYes') {
        getPersonLocation(intent, session, callback);
    } else if (intentName === 'AMAZON.HelpIntent') {
        getWelcomeResponse(callback);
    } else if (intentName === 'AMAZON.StopIntent' || intentName === 'AMAZON.CancelIntent' || intentName === 'receptionThanks') {
        handleSessionEndRequest(callback);
    } else {
        throw new Error('Invalid intent');
    }
}

/**
 * Called when the user ends the session.
 * Is not called when the skill returns shouldEndSession=true.
 */
function onSessionEnded(sessionEndedRequest, session) {
    console.log(`onSessionEnded requestId=${sessionEndedRequest.requestId}, sessionId=${session.sessionId}`);
    // Add cleanup logic here
}


// --------------- Main handler -----------------------

// Route the incoming request based on type (LaunchRequest, IntentRequest,
// etc.) The JSON body of the request is provided in the event parameter.
exports.handler = (event, context, callback) => {
    try {
        console.log(`event.session.application.applicationId=${event.session.application.applicationId}`);

        /**
         * Uncomment this if statement and populate with your skill's application ID to
         * prevent someone else from configuring a skill that sends requests to this function.
         */
        /*
        if (event.session.application.applicationId !== 'amzn1.echo-sdk-ams.app.[unique-value-here]') {
             callback('Invalid Application ID');
        }
        */

        if (event.session.new) {
            onSessionStarted({ requestId: event.request.requestId }, event.session);
        }

        if (event.request.type === 'LaunchRequest') {
            onLaunch(event.request,
                event.session,
                (sessionAttributes, speechletResponse) => {
                    callback(null, buildResponse(sessionAttributes, speechletResponse));
                });
        } else if (event.request.type === 'IntentRequest') {
            onIntent(event.request,
                event.session,
                (sessionAttributes, speechletResponse) => {
                    callback(null, buildResponse(sessionAttributes, speechletResponse));
                });
        } else if (event.request.type === 'SessionEndedRequest') {
            onSessionEnded(event.request, event.session);
            callback();
        }
    } catch (err) {
        callback(err);
    }
};
