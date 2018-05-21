'use strict';

 // --------------- Helpers to build responses which match the structure of the necessary dialog actions -----------------------

function elicitSlot(sessionAttributes, intentName, slots, slotToElicit, message) {
    return {
        sessionAttributes,
        dialogAction: {
            type: 'ElicitSlot',
            intentName,
            slots,
            slotToElicit,
            message,
        },
    };
}

function delegate(sessionAttributes, slots) {
    return {
        sessionAttributes,
        dialogAction: {
            type: 'Delegate',
            slots,
        },
    };
}

function close(sessionAttributes, fulfillmentState, message) {
    return {
        sessionAttributes,
        dialogAction: {
            type: 'Close',
            fulfillmentState,
            message,
        },
    };
}


// ---------------- Helper Functions --------------------------------------------------

function buildValidationResult(isValid, violatedSlot, messageContent) {
    if (messageContent == null) {
        return {
            isValid,
            violatedSlot,
        };
    }
    return {
        isValid,
        violatedSlot,
        message: { 
            contentType: 'PlainText', 
            content: messageContent 
        },
    };
}



 // --------------- Functions that control the bot's behavior -----------------------

function getGreeting(intent, callback) {
    callback(close(
        intent.sessionAttributes, 
        'Fulfilled',
        {
            contentType: 'PlainText',
            content: 'Hello, how may I help you?'
        })
    );
}

function validateTargetName(targetName) {
    const targetNames = ['dr smith', 'smith'];
    if (targetName && targetNames.indexOf(targetName.toLowerCase()) === -1) {
        return buildValidationResult(
            false,
            'user_name',
            `There is no person named ${targetName}, 
                could you please confirm the name of the person you are here to see?`
            );
    }
    return buildValidationResult(true, null, null);
}

function setMeetingInfo(intent, callback) {
    const targetName = intent.currentIntent.slots.target_name;
    // const userName = intent.currentIntent.slots.user_name;
    const userFirstName = intent.currentIntent.slots.first_name;
    const userLastName = intent.currentIntent.slots.last_name;
    const time = intent.currentIntent.slots.time;
    const source = intent.invocationSource;

    if (source === 'DialogCodeHook') {
        const slots = intent.currentIntent.slots;
        const validationResult = validateTargetName(targetName);
        // Check validity of target_name
        if (!validationResult.isValid) {
            slots[`${validationResult.violatedSlot}`] = null;
            callback(elicitSlot(
                intent.sessionAttributes,
                intent.currentIntent.name,
                slots,
                validationResult.violatedSlot,
                validationResult.message)
            );
        }

        const outputSessionAttributes = intent.sessionAttributes || {};
        callback(delegate(
            outputSessionAttributes,
            intent.currentIntent.slots)
        );
    }
    // All slots are satisfied
    const outputSessionAttributes = {
        // "user_name": userName,
        "first_name": userFirstName,
        "last_name": userLastName,
        "target_name": targetName,
        "time": time
    }
    callback(close(
        outputSessionAttributes,
        'Fulfilled',
        {
            contentType: 'PlainText',
            content: `Hi ${userFirstName} ${userLastName}, I have checked you in. 
                please have a seat, ${targetName} will be with you shortly.`
        })
    );
}

function getWaitingTime(intent, session, callback) {
    // const userName = intent.sessionAttributes.user_name;
    const userFirstName = intent.sessionAttributes.first_name;
    const userLastName = intent.sessionAttributes.last_name;
    const targetName = intent.sessionAttributes.target_name;

    callback(close(
        session, 
        'Fulfilled',
        {
            contentType: 'PlainText',
            content: `Hi ${userFirstName} ${userLastName}, ${targetName} is current busy with a another patient, 
                the estimated waiting time is 7 mins`
        })
    );
}

function getAppointmentDuration(intent, session, callback) {
    callback(close(
        session,
        'Fulfilled',
        {
            contentType: 'PlainText',
            content: `Our standard appointments with the Doctor are 15mins long,
                unless you have made prior arrangements`
        })
    );
}

 // --------------- Intents -----------------------

/**
 * Called when the user specifies an intent for this skill.
 */
function dispatch(intent, session, callback) {
    console.log(`dispatch userId=${intent.userId}, intentName=${intent.currentIntent.name}`);

    const intentName = intent.currentIntent.name;

    // Dispatch to your skill's intent handlers
    if (intentName === 'Welcome') {
        return getGreeting(intent, callback);
    } else if (intentName === 'Meeting') {
        return setMeetingInfo(intent, callback);
    } else if (intentName === 'WaitingAppointment') {
        return getWaitingTime(intent, session, callback);
    } else if (intentName === 'appointmentDuration') {
        return getAppointmentDuration(intent, session, callback);
    } else {
        throw new Error(`Intent with name ${intentName} not supported`);    
    }
}

// --------------- Main handler -----------------------

// Route the incoming request based on intent.
// The JSON body of the request is provided in the event slot.
exports.handler = (event, context, callback) => {
    try {
        // By default, treat the user request as coming from the America/New_York time zone.
        process.env.TZ = 'America/New_York';
        console.log(`event.bot.name=${event.bot.name}`);

        dispatch(event, event.session, (response) => callback(null, response));
    } catch (err) {
        callback(err);
    }
};
