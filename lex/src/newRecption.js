// Lambda Function code for Alexa.
// Paste this into your index.js file. 

const Alexa = require("ask-sdk");
const https = require("https");


const invocationName = "reception";

// Session Attributes 
//   Alexa will track attributes for you, by default only during the lifespan of your session.
//   The history[] array will track previous request(s), used for contextual Help/Yes/No handling.
//   Set up DynamoDB persistence to have the skill save and reload these attributes between skill sessions.
function getMemoryAttributes() {   const memoryAttributes = {
       "history":[],

        // The remaining attributes will be useful after DynamoDB persistence is configured
       "launchCount":0,
       "lastUseTimestamp":0

       // "favoriteColor":"",
       // "name":"",
       // "namePronounce":"",
       // "email":"",
       // "mobileNumber":"",
       // "city":"",
       // "state":"",
       // "postcode":"",
       // "birthday":"",
       // "bookmark":0,
       // "wishlist":[],
   };
   return memoryAttributes;
};

const maxHistorySize = 20; // remember only latest 20 intents 


// 1. Intent Handlers =============================================

const AMAZON_CancelIntent_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        return request.type === 'IntentRequest' && request.intent.name === 'AMAZON.CancelIntent' ;
    },
    handle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        const responseBuilder = handlerInput.responseBuilder;
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();


        let say = 'Okay, talk to you later! ';

        return responseBuilder
            .speak(say)
            .withShouldEndSession(true)
            .getResponse();
    },
};

const AMAZON_HelpIntent_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        return request.type === 'IntentRequest' && request.intent.name === 'AMAZON.HelpIntent' ;
    },
    handle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        const responseBuilder = handlerInput.responseBuilder;
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();

        let intents = getCustomIntents();
        let sampleIntent = randomElement(intents);

        let say = 'You asked for help. '; 
        let previousIntent = getPreviousIntent(sessionAttributes);

        if (previousIntent && !handlerInput.requestEnvelope.session.new) {
            say += 'Your last intent was ' + previousIntent + '. ';
        }
        say +=  'I understand  ' + intents.length + ' intents, here something you can ask me, ' + getSampleUtterance(sampleIntent);

        return responseBuilder
            .speak(say)
            .reprompt('try again, ' + say)
            .getResponse();
    },
};

const AMAZON_StopIntent_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        return request.type === 'IntentRequest' && request.intent.name === 'AMAZON.StopIntent' ;
    },
    handle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        const responseBuilder = handlerInput.responseBuilder;
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();


        let say = 'Okay, talk to you later! ';

        return responseBuilder
            .speak(say)
            .withShouldEndSession(true)
            .getResponse();
    },
};

const AMAZON_PauseIntent_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        return request.type === 'IntentRequest' && request.intent.name === 'AMAZON.PauseIntent' ;
    },
    handle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        const responseBuilder = handlerInput.responseBuilder;
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();

        let say = 'Hello from AMAZON.PauseIntent. ';


        return responseBuilder
            .speak(say)
            .reprompt('try again, ' + say)
            .getResponse();
    },
};

const AMAZON_ResumeIntent_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        return request.type === 'IntentRequest' && request.intent.name === 'AMAZON.ResumeIntent' ;
    },
    handle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        const responseBuilder = handlerInput.responseBuilder;
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();

        let say = 'Hello from AMAZON.ResumeIntent. ';


        return responseBuilder
            .speak(say)
            .reprompt('try again, ' + say)
            .getResponse();
    },
};

const comingAlone_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        return request.type === 'IntentRequest' && 
                request.intent.name === 'comingAlone' ;
    },
    handle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        const responseBuilder = handlerInput.responseBuilder;
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();

        // delegate to Alexa to collect all the required slots 
        const currentIntent = request.intent; 
        if (request.dialogState && request.dialogState !== 'COMPLETED') { 
            return handlerInput.responseBuilder
                .addDelegateDirective(currentIntent)
                .getResponse();

        } 
        let say = '';
        
        let _user_name = '';
        let _time = '';
        let _situation = '';
        let _date = '';
        let _target_name = '';

        let slotStatus = '';
        let resolvedSlot;

    //   SLOT: user_name 
        if (request.intent.slots.user_name &&
            request.intent.slots.user_name.value &&
            request.intent.slots.user_name.value !== '?'
        ) {
            const user_name = request.intent.slots.user_name;
            _user_name = user_name.value;

            resolvedSlot = resolveCanonical(user_name);

            if(resolvedSlot != user_name.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot user_name is empty. ';
        }

    //   SLOT: time 
        if (request.intent.slots.time &&
            request.intent.slots.time.value &&
            request.intent.slots.time.value !== '?'
        ) {
            const time = request.intent.slots.time;
            _time = time.value;

            resolvedSlot = resolveCanonical(time);

            if(resolvedSlot != time.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot time is empty. ';
        }

    //   SLOT: situation 
        if (request.intent.slots.situation &&
            request.intent.slots.situation.value &&
            request.intent.slots.situation.value !== '?'
        ) {
            const situation = request.intent.slots.situation;
            _situation = situation.value;

            resolvedSlot = resolveCanonical(situation);

            if(resolvedSlot != situation.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot situation is empty. ';
        }

    //   SLOT: date 
        if (request.intent.slots.date &&
            request.intent.slots.date.value &&
            request.intent.slots.date.value !== '?'
        ) {
            const date = request.intent.slots.date;
            _date = date.value

            resolvedSlot = resolveCanonical(date);

            if(resolvedSlot != date.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot date is empty. ';
        }

    //   SLOT: target_name 
        if (request.intent.slots.target_name &&
            request.intent.slots.target_name.value &&
            request.intent.slots.target_name.value !== '?'
        ) {
            const target_name = request.intent.slots.target_name;
            _target_name = target_name.value;

            resolvedSlot = resolveCanonical(target_name);

            if(resolvedSlot != target_name.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot target_name is empty. ';
        }


        if (_target_name != '') {
            say = `Hi ${_user_name}, I have checked you in. Please have a seat in the waiting area until your name is called`;
        } else {
            let _expectedTargetName = 'Dr Smith'; // need to be retrieved from DB
            say = `You may forget your Dr name. Not a problem ${_user_name}, I can see that your appointment today is with ${_expectedTargetName}. You are all checked in.
                    Please have a seat in the waiting area until your name is called.`
        }

        return responseBuilder
            .speak(say)
            .reprompt('try again, ' + say)
            .getResponse();
    },
};

const waiting_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();
        let _previousIntent = getPreviousIntent(sessionAttributes).IntentRequest;
        
        return request.type === 'IntentRequest' && request.intent.name === 'waiting';
    },
    handle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        const responseBuilder = handlerInput.responseBuilder;
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();

        let _expectedTime = Math.floor((Math.random()*30) + 5); // need to be retrieved from DB

        let previousIntent = getPreviousIntent(sessionAttributes);
        console.log('data: ' + JSON.stringify(previousIntent.slots, null, 2))
        console.log('test: ' + JSON.stringify(previousIntent.slots[3]));
        
        let _target_name = previousIntent.slots[1].target_name;
        let _user_name = previousIntent.slots[2].user_name;
        

        let say = '';

        if(_expectedTime < 10) {
            say = `${_user_name}, you are next to see ${_target_name}. He will be around ${_expectedTime} minutes.`;
        } else {
            say = `${_user_name}, I am afraid ${_target_name} is running late. He will be around ${_expectedTime} minutes. I am sorry if this causes you any inconvenience.`;
        }
        
        return responseBuilder
            .speak(say)
            .reprompt('try again, ' + say)
            .getResponse();
    },
};

const appointmentDuration_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        return request.type === 'IntentRequest' && request.intent.name === 'appointmentDuration' ;
    },
    handle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        const responseBuilder = handlerInput.responseBuilder;
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();

        let say = '';

        let slotStatus = '';
        let resolvedSlot;

    //   SLOT: situation 
        if (request.intent.slots.situation &&
            request.intent.slots.situation.value &&
            request.intent.slots.situation.value !== '?'
        ) {
            const situation = request.intent.slots.situation;
            slotStatus += ' slot situation was heard as ' + situation.value + '. ';

            resolvedSlot = resolveCanonical(situation);

            if(resolvedSlot != situation.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot situation is empty. ';
        }


        say = `Our standard appointments are 15mintes long. If you require more time with the Dr you will need to make a double booking.`;


        return responseBuilder
            .speak(say)
            .reprompt('try again, ' + say)
            .getResponse();
    },
};

const comingWithCompanion_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        return request.type === 'IntentRequest' && request.intent.name === 'comingWithCompanion' ;
    },
    handle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        const responseBuilder = handlerInput.responseBuilder;
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();

        // delegate to Alexa to collect all the required slots 
        const currentIntent = request.intent; 
        if (request.dialogState && request.dialogState !== 'COMPLETED') { 
            return handlerInput.responseBuilder
                .addDelegateDirective(currentIntent)
                .getResponse();

        } 
        let say = '';

        let _user_name = '';
        let _target_name = '';
        let _companion_name = '';
        let _relationship = '';

        let slotStatus = '';
        let resolvedSlot;

    //   SLOT: user_name 
        if (request.intent.slots.user_name &&
            request.intent.slots.user_name.value &&
            request.intent.slots.user_name.value !== '?'
        ) {
            const user_name = request.intent.slots.user_name;
            _user_name = user_name.value;

            resolvedSlot = resolveCanonical(user_name);

            if(resolvedSlot != user_name.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot user_name is empty. ';
        }

    //   SLOT: time 
        if (request.intent.slots.time &&
            request.intent.slots.time.value &&
            request.intent.slots.time.value !== '?'
        ) {
            const time = request.intent.slots.time;
            slotStatus += ' slot time was heard as ' + time.value + '. ';

            resolvedSlot = resolveCanonical(time);

            if(resolvedSlot != time.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot time is empty. ';
        }

    //   SLOT: situation 
        if (request.intent.slots.situation &&
            request.intent.slots.situation.value &&
            request.intent.slots.situation.value !== '?'
        ) {
            const situation = request.intent.slots.situation;
            slotStatus += ' slot situation was heard as ' + situation.value + '. ';

            resolvedSlot = resolveCanonical(situation);

            if(resolvedSlot != situation.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot situation is empty. ';
        }

    //   SLOT: date 
        if (request.intent.slots.date &&
            request.intent.slots.date.value &&
            request.intent.slots.date.value !== '?'
        ) {
            const date = request.intent.slots.date;
            slotStatus += ' slot date was heard as ' + date.value + '. ';

            resolvedSlot = resolveCanonical(date);

            if(resolvedSlot != date.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot date is empty. ';
        }

    //   SLOT: target_name 
        if (request.intent.slots.target_name &&
            request.intent.slots.target_name.value &&
            request.intent.slots.target_name.value !== '?'
        ) {
            const target_name = request.intent.slots.target_name;
            _target_name = target_name.value;

            resolvedSlot = resolveCanonical(target_name);

            if(resolvedSlot != target_name.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot target_name is empty. ';
        }

    //   SLOT: companion_name 
        if (request.intent.slots.companion_name &&
            request.intent.slots.companion_name.value &&
            request.intent.slots.companion_name.value !== '?'
        ) {
            const companion_name = request.intent.slots.companion_name;
            _companion_name = companion_name.value;

            resolvedSlot = resolveCanonical(companion_name);

            if(resolvedSlot != companion_name.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot companion_name is empty. ';
        }

    //   SLOT: relationship 
        if (request.intent.slots.relationship &&
            request.intent.slots.relationship.value &&
            request.intent.slots.relationship.value !== '?'
        ) {
            const relationship = request.intent.slots.relationship;
            slotStatus += ' slot relationship was heard as ' + relationship.value + '. ';

            resolvedSlot = resolveCanonical(relationship);

            if(resolvedSlot != relationship.value) {
                slotStatus += ' which resolved to ' + resolvedSlot + '. '; 
            }
        } else {
            slotStatus += ' slot relationship is empty. ';
        }

        console.log(_target_name);
        if(_target_name != '') {
            say = `${_companion_name}, I have checked ${_user_name} in. Please have a seat in the waiting area until his name is called`;
        }else{
            let _expectedTargetName = 'Dr Smith';
            say = `That’s fine ${_companion_name}, I can see that ${_user_name} is seeing ${_expectedTargetName} today. I have checked ${_user_name} in.
                    Please have a seat in the waiting area until his name is called.`
        }
        

        return responseBuilder
            .speak(say)
            .reprompt('try again, ' + say)
            .getResponse();
    },
};

const AMAZON_YesIntent_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;

        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();
        let _previousIntent = getPreviousIntent(sessionAttributes).IntentRequest;

        return request.type === 'IntentRequest' && request.intent.name === 'AMAZON.YesIntent' && _previousIntent === 'comingAlone';
    },
    handle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        const responseBuilder = handlerInput.responseBuilder;
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();

        let say = `See “booking appointment” scenario`;

        return responseBuilder
            .speak(say)
            .reprompt('try again, ' + say)
            .getResponse();
    },
};

const AMAZON_NoIntent_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;

        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();
        let _previousIntent = getPreviousIntent(sessionAttributes).IntentRequest;

        return request.type === 'IntentRequest' && request.intent.name === 'AMAZON.NoIntent' && _previousIntent === 'comingAlone';
    },
    handle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        const responseBuilder = handlerInput.responseBuilder;
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();

        let say = 'ok, then';

        return responseBuilder
            .speak(say)
            .reprompt('try again, ' + say)
            .getResponse();
    },
};

const makeDoubleBooking_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        return request.type === 'IntentRequest' && request.intent.name === 'makeDoubleBooking' ;
    },
    handle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        const responseBuilder = handlerInput.responseBuilder;
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes();

        let say = 'Hello from makeDoubleBooking. ';


        return responseBuilder
            .speak(say)
            .reprompt('try again, ' + say)
            .getResponse();
    },
};

const LaunchRequest_Handler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        return request.type === 'LaunchRequest';
    },
    handle(handlerInput) {
        const responseBuilder = handlerInput.responseBuilder;

        let say = 'Hi there, my name is Ever. How may I help you?';

        let skillTitle = capitalize(invocationName);


        return responseBuilder
            .speak(say)
            .reprompt('try again, ' + say)
            .withStandardCard('Welcome!', 
              'Hello!\nThis is a card for your skill, ' + skillTitle,
               welcomeCardImg.smallImageUrl, welcomeCardImg.largeImageUrl)
            .getResponse();
    },
};

const SessionEndedHandler =  {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        return request.type === 'SessionEndedRequest';
    },
    handle(handlerInput) {
        console.log(`Session ended with reason: ${handlerInput.requestEnvelope.request.reason}`);
        return handlerInput.responseBuilder.getResponse();
    }
};

const ErrorHandler =  {
    canHandle() {
        return true;
    },
    handle(handlerInput, error) {
        const request = handlerInput.requestEnvelope.request;

        console.log(`Error handled: ${error.message}`);
        console.log(`Original Request was: ${JSON.stringify(request, null, 2)}`);

        return handlerInput.responseBuilder
            .speak('Sorry, I can not understand the command.  Please say again.')
            .reprompt('Sorry, I can not understand the command.  Please say again.')
            .getResponse();
    }
};


// 2. Constants ===========================================================================

    // Here you can define static data, to be used elsewhere in your code.  For example: 
    //    const myString = "Hello World";
    //    const myArray  = [ "orange", "grape", "strawberry" ];
    //    const myObject = { "city": "Boston",  "state":"Massachusetts" };

const APP_ID = undefined;  // TODO replace with your Skill ID (OPTIONAL).

// 3.  Helper Functions ===================================================================

function capitalize(myString) {

     return myString.replace(/(?:^|\s)\S/g, function(a) { return a.toUpperCase(); }) ;
}

 
function randomElement(myArray) { 
    return(myArray[Math.floor(Math.random() * myArray.length)]); 
} 
 
 
function resolveCanonical(slot){ 
    let canonical = ''; 
    if (slot.hasOwnProperty('resolutions')) { 
        canonical = slot.resolutions.resolutionsPerAuthority[0].values[0].value.name; 
    } else { 
        canonical = slot.value; 
    } 
 
    return canonical; 
} 
 
 
function getSlotValues(filledSlots) { 
    const slotValues = {}; 
 
    console.log(`The filled slots: ${JSON.stringify(filledSlots)}`); 
    Object.keys(filledSlots).forEach((item) => { 
        const name  = filledSlots[item].name;
 
        if (filledSlots[item] && 
            filledSlots[item].resolutions && 
            filledSlots[item].resolutions.resolutionsPerAuthority[0] && 
            filledSlots[item].resolutions.resolutionsPerAuthority[0].status && 
            filledSlots[item].resolutions.resolutionsPerAuthority[0].status.code) { 
            switch (filledSlots[item].resolutions.resolutionsPerAuthority[0].status.code) { 
                case 'ER_SUCCESS_MATCH': 
                    slotValues[name] = { 
                        synonym: filledSlots[item].value, 
                        resolved: filledSlots[item].resolutions.resolutionsPerAuthority[0].values[0].value.name, 
                        isValidated: true 
                    }; 
                    break; 
                case 'ER_SUCCESS_NO_MATCH': 
                    slotValues[name] = { 
                        synonym: filledSlots[item].value, 
                        resolved: filledSlots[item].value, 
                        isValidated: false 
                    }; 
                    break; 
                default: 
                    break; 
            } 
        } else { 
            slotValues[name] = { 
                synonym: filledSlots[item].value, 
                resolved: filledSlots[item].value, 
                isValidated: false 
            }; 
        } 
    }, this); 
    
    return slotValues; 
} 
 
function supportsDisplay(handlerInput) // returns true if the skill is running on a device with a display (Echo Show, Echo Spot, etc.) 
{                                      //  Enable your skill for display as shown here: https://alexa.design/enabledisplay 
    const hasDisplay = 
        handlerInput.requestEnvelope.context && 
        handlerInput.requestEnvelope.context.System && 
        handlerInput.requestEnvelope.context.System.device && 
        handlerInput.requestEnvelope.context.System.device.supportedInterfaces && 
        handlerInput.requestEnvelope.context.System.device.supportedInterfaces.Display; 
 
    return hasDisplay; 
} 
 
 
const welcomeCardImg = { 
    smallImageUrl: "https://s3.amazonaws.com/skill-images-789/cards/card_plane720_480.png", 
    largeImageUrl: "https://s3.amazonaws.com/skill-images-789/cards/card_plane1200_800.png" 
 
 
}; 
 
const DisplayImg1 = { 
    title: 'Jet Plane', 
    url: 'https://s3.amazonaws.com/skill-images-789/display/plane340_340.png' 
}; 
const DisplayImg2 = { 
    title: 'Starry Sky', 
    url: 'https://s3.amazonaws.com/skill-images-789/display/background1024_600.png' 
 
}; 
 
function getCustomIntents() { 
    const modelIntents = interactionModel.interactionModel.languageModel.intents; 
 
    let customIntents = []; 
 
 
    for (let i = 0; i < modelIntents.length; i++) { 
 
        if(modelIntents[i].name.substring(0,7) != "AMAZON." && modelIntents[i].name !== "LaunchRequest" ) { 
            customIntents.push(modelIntents[i]); 
        } 
    } 
    return customIntents; 
} 
 
function getSampleUtterance(intent) { 
 
 
    return randomElement(intent.samples); 
 
} 
 
function getPreviousIntent(attrs) { 
 
    if (attrs.history && attrs.history.length > 1) { 
        return attrs.history[attrs.history.length - 2]; 
 
    } else { 
        return false; 
    } 
 
} 
 
function timeDelta(t1, t2) { 
 
    const dt1 = new Date(t1); 
    const dt2 = new Date(t2); 
    const timeSpanMS = dt2.getTime() - dt1.getTime(); 
    const span = { 
        "timeSpanMIN": Math.floor(timeSpanMS / (1000 * 60 )), 
        "timeSpanHR": Math.floor(timeSpanMS / (1000 * 60 * 60)), 
        "timeSpanDAY": Math.floor(timeSpanMS / (1000 * 60 * 60 * 24)), 
        "timeSpanDesc" : "" 
    }; 
 
 
    if (span.timeSpanHR < 2) { 
        span.timeSpanDesc = span.timeSpanMIN + " minutes"; 
    } else if (span.timeSpanDAY < 2) { 
        span.timeSpanDesc = span.timeSpanHR + " hours"; 
    } else { 
        span.timeSpanDesc = span.timeSpanDAY + " days"; 
    } 
 
 
    return span; 
 
} 
 
 
const InitMemoryAttributesInterceptor = { 
    process(handlerInput) { 
        let sessionAttributes = {}; 
        if(handlerInput.requestEnvelope.session['new']) { 
 
            sessionAttributes = handlerInput.attributesManager.getSessionAttributes(); 
 
            let memoryAttributes = getMemoryAttributes(); 
 
            if(Object.keys(sessionAttributes).length === 0) { 
 
                Object.keys(memoryAttributes).forEach(function(key) {  // initialize all attributes from global list 
 
                    sessionAttributes[key] = memoryAttributes[key]; 
 
                }); 
 
            } 
            handlerInput.attributesManager.setSessionAttributes(sessionAttributes); 
 
 
        } 
    } 
}; 
 
const RequestHistoryInterceptor = { 
    process(handlerInput) { 
 
        const thisRequest = handlerInput.requestEnvelope.request; 
        let sessionAttributes = handlerInput.attributesManager.getSessionAttributes(); 
 
        let history = sessionAttributes['history'] || []; 
 
        let IntentRequest = {}; 
        if (thisRequest.type === 'IntentRequest' ) { 
 
            let slots = []; 
 
            IntentRequest = { 
                'IntentRequest' : thisRequest.intent.name 
            }; 
 
            if (thisRequest.intent.slots) { 
 
                for (let slot in thisRequest.intent.slots) { 
                    let slotObj = {}; 
                    slotObj[slot] = thisRequest.intent.slots[slot].value; 
                    slots.push(slotObj); 
                } 
 
                IntentRequest = { 
                    'IntentRequest' : thisRequest.intent.name, 
                    'slots' : slots 
                }; 
 
            } 
 
        } else { 
            IntentRequest = {'IntentRequest' : thisRequest.type}; 
        } 
        if(history.length > maxHistorySize - 1) { 
            history.shift(); 
        } 
        history.push(IntentRequest); 
 
        handlerInput.attributesManager.setSessionAttributes(sessionAttributes); 
 
    } 
 
}; 
 
 
 
 
const RequestPersistenceInterceptor = { 
    process(handlerInput) { 
 
        if(handlerInput.requestEnvelope.session['new']) { 
 
            return new Promise((resolve, reject) => { 
 
                handlerInput.attributesManager.getPersistentAttributes() 
 
                    .then((sessionAttributes) => { 
                        sessionAttributes = sessionAttributes || {}; 
 
 
                        sessionAttributes['launchCount'] += 1; 
 
                        handlerInput.attributesManager.setSessionAttributes(sessionAttributes); 
 
                        handlerInput.attributesManager.savePersistentAttributes() 
                            .then(() => { 
                                resolve(); 
                            }) 
                            .catch((err) => { 
                                reject(err); 
                            }); 
                    }); 
 
            }); 
 
        } // end session['new'] 
    } 
}; 
 
 
const ResponsePersistenceInterceptor = { 
    process(handlerInput, responseOutput) { 
 
        const ses = (typeof responseOutput.shouldEndSession == "undefined" ? true : responseOutput.shouldEndSession); 
 
        if(ses || handlerInput.requestEnvelope.request.type == 'SessionEndedRequest') { // skill was stopped or timed out 
 
            let sessionAttributes = handlerInput.attributesManager.getSessionAttributes(); 
 
            sessionAttributes['lastUseTimestamp'] = new Date(handlerInput.requestEnvelope.request.timestamp).getTime(); 
 
            handlerInput.attributesManager.setPersistentAttributes(sessionAttributes); 
 
            return new Promise((resolve, reject) => { 
                handlerInput.attributesManager.savePersistentAttributes() 
                    .then(() => { 
                        resolve(); 
                    }) 
                    .catch((err) => { 
                        reject(err); 
                    }); 
 
            }); 
 
        } 
 
    } 
}; 
 
 
 
// 4. Exports handler function and setup ===================================================
const skillBuilder = Alexa.SkillBuilders.standard();
exports.handler = skillBuilder
    .addRequestHandlers(
        AMAZON_CancelIntent_Handler, 
        AMAZON_HelpIntent_Handler, 
        AMAZON_StopIntent_Handler, 
        AMAZON_PauseIntent_Handler, 
        AMAZON_ResumeIntent_Handler, 
        comingAlone_Handler, 
        waiting_Handler, 
        appointmentDuration_Handler, 
        comingWithCompanion_Handler, 
        AMAZON_YesIntent_Handler, 
        AMAZON_NoIntent_Handler, 
        makeDoubleBooking_Handler, 
        LaunchRequest_Handler, 
        SessionEndedHandler
    )
    .addErrorHandlers(ErrorHandler)
    .addRequestInterceptors(InitMemoryAttributesInterceptor)
    .addRequestInterceptors(RequestHistoryInterceptor)

 // .addRequestInterceptors(RequestPersistenceInterceptor)
 // .addResponseInterceptors(ResponsePersistenceInterceptor)

 // .withTableName("askMemorySkillTable")
 // .withAutoCreateTable(true)

    .lambda();


// End of Skill code -------------------------------------------------------------
// Static Language Model for reference

const interactionModel = {
    "interactionModel": {
      "languageModel": {
        "invocationName": "reception",
        "intents": [
          {
            "name": "AMAZON.CancelIntent",
            "samples": []
          },
          {
            "name": "AMAZON.HelpIntent",
            "samples": []
          },
          {
            "name": "AMAZON.StopIntent",
            "samples": []
          },
          {
            "name": "AMAZON.PauseIntent",
            "samples": []
          },
          {
            "name": "AMAZON.ResumeIntent",
            "samples": []
          },
          {
            "name": "comingAlone",
            "slots": [
              {
                "name": "user_name",
                "type": "AMAZON.US_FIRST_NAME",
                "samples": [
                  "My name is {user_name} I have {situation} at {time} but I cannot remember which Doctor I am seeing",
                  "{user_name}",
                  "my name is {user_name}",
                  "my name is {user_name} i have {situation} with {target_name} at {time}"
                ]
              },
              {
                "name": "time",
                "type": "AMAZON.TIME",
                "samples": [
                  "i have {situation} with {target_name} at {time}",
                  "I have {situation} at {time}",
                  "{time}",
                  "my name is {user_name} i have {situation} with {target_name} at {time}"
                ]
              },
              {
                "name": "situation",
                "type": "reception_situation"
              },
              {
                "name": "date",
                "type": "AMAZON.DATE"
              },
              {
                "name": "target_name",
                "type": "LIST_OF_PEOPLE"
              }
            ],
            "samples": [
              "Um can you help me check in for my Doctors {situation} ",
              "Hi I have {situation} to see {target_name} {date}",
              "Hi I have {situation} at {time}",
              "Hi my name is {user_name} and I have a Drs {situation}",
              "I have {situation}",
              "Hello I'm here for {situation}",
              "Hello I'm here to see {target_name} at {time}",
              "Hi my name is {user_name} and I have {situation} with {target_name} {date} at {time}"
            ]
          },
          {
            "name": "waiting",
            "slots": [
              {
                "name": "target_name",
                "type": "LIST_OF_PEOPLE"
              }
            ],
            "samples": [
              "is {target_name} running on time",
              "Can you please let me know how long I need to wait"
            ]
          },
          {
            "name": "appointmentDuration",
            "slots": [
              {
                "name": "situation",
                "type": "reception_situation"
              }
            ],
            "samples": [
              "Can you please tell me how long the {situation} will take"
            ]
          },
          {
            "name": "comingWithCompanion",
            "slots": [
              {
                "name": "user_name",
                "type": "AMAZON.US_FIRST_NAME"
              },
              {
                "name": "time",
                "type": "AMAZON.TIME"
              },
              {
                "name": "situation",
                "type": "reception_situation"
              },
              {
                "name": "date",
                "type": "AMAZON.DATE"
              },
              {
                "name": "target_name",
                "type": "LIST_OF_PEOPLE"
              },
              {
                "name": "companion_name",
                "type": "AMAZON.US_FIRST_NAME"
              },
              {
                "name": "relationship",
                "type": "relationshipWithCompanion",
                "samples": [
                  "correct i am {relationship}",
                  "i am his {relationship}",
                  "yes i am his {relationship}"
                ]
              }
            ],
            "samples": [
              "Hi my name is {companion_name} and this is my son {user_name} He has an appointment with {target_name} today at {time}"
            ]
          },
          {
            "name": "AMAZON.YesIntent",
            "samples": []
          },
          {
            "name": "AMAZON.NoIntent",
            "samples": []
          },
          {
            "name": "makeDoubleBooking",
            "slots": [],
            "samples": [
              "Can I make a double booking now"
            ]
          },
          {
            "name": "LaunchRequest"
          }
        ],
        "types": [
          {
            "name": "LIST_OF_PEOPLE",
            "values": [
              {
                "name": {
                  "value": "Smith"
                }
              },
              {
                "name": {
                  "value": "Dr Smith"
                }
              },
              {
                "name": {
                  "value": "Bruce"
                }
              },
              {
                "name": {
                  "value": "Eddie"
                }
              },
              {
                "name": {
                  "value": "Ahn"
                }
              }
            ]
          },
          {
            "name": "reception_situation",
            "values": [
              {
                "name": {
                  "value": "an appointment"
                }
              },
              {
                "name": {
                  "value": "appointment"
                }
              }
            ]
          },
          {
            "name": "relationshipWithCompanion",
            "values": [
              {
                "name": {
                  "value": "father"
                }
              },
              {
                "name": {
                  "value": "mother"
                }
              }
            ]
          }
        ]
      },
      "dialog": {
        "intents": [
          {
            "name": "comingAlone",
            "confirmationRequired": false,
            "prompts": {},
            "slots": [
              {
                "name": "user_name",
                "type": "AMAZON.US_FIRST_NAME",
                "confirmationRequired": false,
                "elicitationRequired": true,
                "prompts": {
                  "elicitation": "Elicit.Slot.951963927494.1064804587532"
                }
              },
              {
                "name": "time",
                "type": "AMAZON.TIME",
                "confirmationRequired": false,
                "elicitationRequired": true,
                "prompts": {
                  "elicitation": "Elicit.Slot.951963927494.1153872148180"
                }
              },
              {
                "name": "situation",
                "type": "reception_situation",
                "confirmationRequired": false,
                "elicitationRequired": false,
                "prompts": {}
              },
              {
                "name": "date",
                "type": "AMAZON.DATE",
                "confirmationRequired": false,
                "elicitationRequired": false,
                "prompts": {}
              },
              {
                "name": "target_name",
                "type": "LIST_OF_PEOPLE",
                "confirmationRequired": false,
                "elicitationRequired": false,
                "prompts": {}
              }
            ]
          },
          {
            "name": "comingWithCompanion",
            "confirmationRequired": false,
            "prompts": {},
            "slots": [
              {
                "name": "user_name",
                "type": "AMAZON.US_FIRST_NAME",
                "confirmationRequired": false,
                "elicitationRequired": false,
                "prompts": {}
              },
              {
                "name": "time",
                "type": "AMAZON.TIME",
                "confirmationRequired": false,
                "elicitationRequired": false,
                "prompts": {}
              },
              {
                "name": "situation",
                "type": "reception_situation",
                "confirmationRequired": false,
                "elicitationRequired": false,
                "prompts": {}
              },
              {
                "name": "date",
                "type": "AMAZON.DATE",
                "confirmationRequired": false,
                "elicitationRequired": false,
                "prompts": {}
              },
              {
                "name": "target_name",
                "type": "LIST_OF_PEOPLE",
                "confirmationRequired": false,
                "elicitationRequired": false,
                "prompts": {}
              },
              {
                "name": "companion_name",
                "type": "AMAZON.US_FIRST_NAME",
                "confirmationRequired": false,
                "elicitationRequired": false,
                "prompts": {}
              },
              {
                "name": "relationship",
                "type": "relationshipWithCompanion",
                "confirmationRequired": false,
                "elicitationRequired": true,
                "prompts": {
                  "elicitation": "Elicit.Slot.911439274134.107113119487"
                }
              }
            ]
          }
        ]
      },
      "prompts": [
        {
          "id": "Elicit.Slot.1494860541113.149548559977",
          "variations": [
            {
              "type": "PlainText",
              "value": "Certainly can you please tell me your name the Dr you are seeing and the time of your appointment?"
            },
            {
              "type": "PlainText",
              "value": "Thank you, can you please confirm your name, the name of the person you are here to see and your appointment time?"
            },
            {
              "type": "PlainText",
              "value": "who do you need to meet?"
            }
          ]
        },
        {
          "id": "Elicit.Slot.951963927494.1064804587532",
          "variations": [
            {
              "type": "PlainText",
              "value": "Certainly, can you please tell me your name, the Dr you are seeing and the time of your appointment?"
            },
            {
              "type": "PlainText",
              "value": "Thank you, can you please confirm your name, the name of the person you are here to see and your appointment time?"
            },
            {
              "type": "PlainText",
              "value": "can i get your name?"
            }
          ]
        },
        {
          "id": "Elicit.Slot.951963927494.1153872148180",
          "variations": [
            {
              "type": "PlainText",
              "value": "Hi {user_name} , can you please tell me who you are seeing and the time of your appointment?"
            },
            {
              "type": "PlainText",
              "value": "Thank you, can you please confirm your name, the name of the person you are here to see and your appointment time?"
            },
            {
              "type": "PlainText",
              "value": "when do you have an appointment?"
            }
          ]
        },
        {
          "id": "Elicit.Slot.911439274134.107113119487",
          "variations": [
            {
              "type": "PlainText",
              "value": "Hi {companion_name} , just to clarify, are you checking in on behalf of your son, {user_name}"
            }
          ]
        }
      ]
    }
  };