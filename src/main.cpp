#include <Arduino.h>
#include <M5Cardputer.h>
#include <Preferences.h>
#include <ctype.h>
#include <math.h>

// ============================================================
// Morseduino - M5Stack Cardputer ADV
//
// Robert Rayner, M0YNW
//
// Cardputer ADV version
//
// G1 = radio keying output
// Built-in speaker = CW sidetone
//
// Controls:
//
// Normal mode:
//   Type characters = transmit Morse
//   [ / ]           = decrease / increase WPM
//   TAB             = settings
//   BACKSPACE       = delete last character (not currectly functional)
//
// Menu mode:
//   UP / DOWN        = select item
//   LEFT / RIGHT     = change value
//   ENTER            = select
//   ESC              = return
//
// Settings are saved in ESP32 NVS flash.
// ============================================================

// ============================================================
// Function prototypes
// ============================================================

void updateDisplay();
void updateMenuDisplay();

void updateWpmTiming();
const char* getMorseCode(char c);

void loadSettings();
void saveSettings();

void keyDown();
void keyUp();

void transmitMorse(const char* morse);

void clearText();

void changeWPM(int amount);
void changeMenuValue(int amount);
void selectMenuItem();

void processTransmitCharacter(char c);

void handleTransmitKeyboard();
void handleMenuKeyboard();

void updateCannedDisplay();
void handleCannedKeyboard();

void updateAudioInDisplay();
void handleAudioInKeyboard();

void updateCallsignDisplay();
void handleCallsignKeyboard();
void updateCannedMessages();

void enterReceiveMode();
void leaveReceiveMode();
void updateReceiveDisplay();
void updateReceiveText();
void updateMorseIndicator();
void updateToneIndicator();
void handleReceiveKeyboard();

bool readMicAudioBlock(int16_t* samples, size_t count);
bool readExternalAudioBlock(int16_t* samples, size_t count);

float measureToneRatio(const int16_t* samples, size_t count, int frequency);
bool detectCWTone(const int16_t* samples, size_t count);
void updateRxFrequencyTracking(const int16_t* samples, size_t count);

void processDecoder();
char decodeMorse(const String& code);
void commitDecodedCharacter();

// ============================================================
// Hardware
// ============================================================

const int keyPin = 1;              // G1

int sidetoneFrequency = 700;

// ============================================================
// Morse speed
// ============================================================

int wpm = 20;

const int minimumWPM = 5;
const int maximumWPM = 40;

// ============================================================
// Audio In
//=============================================================

// ============================================================
// Audio
// ============================================================

int speakerVolume = 180;

const int minimumVolume = 0;
const int maximumVolume = 255;

const int minimumTone = 300;
const int maximumTone = 1200;

const int toneStep = 50;


// ============================================================
// RX frequency control
// ============================================================

enum RxFrequencyMode
{
    RX_FREQ_AUTO,
    RX_FREQ_MANUAL
};

RxFrequencyMode rxFrequencyMode = RX_FREQ_AUTO;

// Manual lock frequency, used only in MANUAL mode.
int manualRxFrequency = 700;

// Current frequency used by the detector.
// In AUTO this is updated by the scanner.
// In MANUAL it follows manualRxFrequency.
int rxToneFrequency = 700;

const int minimumRxFrequency = 300;
const int maximumRxFrequency = 1200;
const int rxFrequencyStep = 25;

// AUTO scan parameters.
const int autoScanStart = 400;
const int autoScanEnd = 1000;
const int autoScanStep = 25;

// Frequency candidate must persist before we adopt it.
int autoCandidateFrequency = 700;
uint8_t autoCandidateCount = 0;
const uint8_t autoCandidateRequired = 5;

// Lock state.  Once a CW tone is acquired, only a small frequency
// window is searched until the signal has disappeared for a while.
bool autoFrequencyLocked = false;
unsigned long lastAutoToneSeen = 0;
const unsigned long autoUnlockDelay = 1500;

// Minimum Goertzel ratio needed before AUTO considers a frequency valid.
float autoFrequencyThreshold = 0.12f;

// ============================================================
// Morse timing
// ============================================================

int dotDuration;
int dashDuration;
int elementSpace;
int letterSpace;
int wordSpace;

// ============================================================
// Morse lookup table
// ============================================================

const char* morseCodeMap[40] = {


// A-J
".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---",

// K-T
"-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-",

// U-Z
"..-", "...-", ".--", "-..-", "-.--", "--..",

// 0-5
"-----", ".----", "..---", "...--", "....-", ".....",

// 6-9
"-....", "--...", "---..", "----.",

// /
"-..-.",

// ?
"..--..",

// =
"-...-",

// .
".-.-.-"


};

// ============================================================
// Preferences
// ============================================================

Preferences preferences;

// ============================================================
// Application modes
// ============================================================

enum AppMode
{
MODE_TRANSMIT,
MODE_MENU,
MODE_CANNED,
MODE_AUDIO_IN,
MODE_CALLSIGN,
MODE_RECEIVE
};

AppMode appMode = MODE_TRANSMIT;

// ============================================================
// Menu
// ============================================================

enum MenuItem
{
MENU_WPM,
MENU_VOLUME,
MENU_TONE,
MENU_RX_MODE,
MENU_RX_FREQUENCY,
MENU_AUDIO_IN,
MENU_CALLSIGN,
MENU_CLEAR,
MENU_SAVE,
MENU_COUNT
};

int menuSelection = MENU_WPM;


// ============================================================
// Station settings
// ============================================================

String callsign = "MYCALL";
String callsignEditBuffer = "MYCALL";
bool callsignReplaceOnType = true;

const int maximumCallsignLength = 12;


// ============================================================
// Audio in
// ============================================================

enum AudioSource
{
    AUDIO_MIC,
    AUDIO_EXTERNAL
};

AudioSource audioSource = AUDIO_MIC;
int audioSelection = AUDIO_MIC;


// ============================================================
// CW decoder
// ============================================================

static constexpr size_t RX_BLOCK_SAMPLES = 80;   // 10 ms at 8 kHz
static constexpr uint32_t RX_SAMPLE_RATE = 8000;

int16_t rxAudioBlock[RX_BLOCK_SAMPLES];

// Goertzel tone-energy ratio threshold.
float decoderToneThreshold = 0.18f;

bool decoderToneState = false;
bool decoderCandidateState = false;
uint8_t decoderCandidateCount = 0;

unsigned long decoderStateStart = 0;
unsigned long decoderSilenceStart = 0;

String rxMorseBuffer;
String rxText;

bool decoderWordSpaceAdded = false;

// Future external I2C ADC hook. Leave false until the interface is fitted.
bool externalAudioReady = false;


// ============================================================
// Text buffer
// ============================================================

String txBuffer;


// ============================================================
// Canned messages
// ============================================================

String cannedMessages[6];

const int cannedMessageCount = 6;

int cannedSelection = 0;


// ============================================================
// Build canned messages
// ============================================================

void updateCannedMessages()
{
    cannedMessages[0] = "CQ TEST " + callsign;
    cannedMessages[1] = callsign;
    cannedMessages[2] = "599 TU";
    cannedMessages[3] = "NAME";
    cannedMessages[4] = "QTH";
    cannedMessages[5] = "73 SK";
}

// ============================================================
// Update Morse timing
// ============================================================

void updateWpmTiming()
{
dotDuration = 1200 / wpm;


dashDuration = dotDuration * 3;

elementSpace = dotDuration;

letterSpace = dotDuration * 3;

wordSpace = dotDuration * 7;


}

// ============================================================
// Get Morse code
// ============================================================

const char* getMorseCode(char c)
{
if (c >= 'A' && c <= 'Z')
return morseCodeMap[c - 'A'];


if (c >= '0' && c <= '9')
    return morseCodeMap[26 + (c - '0')];

if (c == '/')
    return morseCodeMap[36];

if (c == '?')
    return morseCodeMap[37];

if (c == '=')
    return morseCodeMap[38];

if (c == '.')
    return morseCodeMap[39];

return "";


}

// ============================================================
// Load settings
// ============================================================

void loadSettings()
{
preferences.begin("morseduino", true);


wpm = preferences.getInt(
    "wpm",
    20
);

speakerVolume = preferences.getInt(
    "volume",
    180
);

sidetoneFrequency = preferences.getInt(
    "tone",
    700
);

rxFrequencyMode = static_cast<RxFrequencyMode>(
    preferences.getInt("rxMode", RX_FREQ_AUTO)
);

if (rxFrequencyMode != RX_FREQ_AUTO &&
    rxFrequencyMode != RX_FREQ_MANUAL)
{
    rxFrequencyMode = RX_FREQ_AUTO;
}

manualRxFrequency = preferences.getInt(
    "rxFreq",
    700
);

manualRxFrequency = constrain(
    manualRxFrequency,
    minimumRxFrequency,
    maximumRxFrequency
);

rxToneFrequency = manualRxFrequency;

audioSource = static_cast<AudioSource>(
    preferences.getInt("audio", AUDIO_MIC)
);

if (audioSource != AUDIO_MIC && audioSource != AUDIO_EXTERNAL)
{
    audioSource = AUDIO_MIC;
}

audioSelection = static_cast<int>(audioSource);

callsign = preferences.getString(
    "callsign",
    "MYCALL"
);

callsign.trim();
callsign.toUpperCase();

if (callsign.length() == 0)
{
    callsign = "MYCALL";
}

if (callsign.length() > maximumCallsignLength)
{
    callsign = callsign.substring(0, maximumCallsignLength);
}

callsignEditBuffer = callsign;

preferences.end();


// Safety limits

wpm = constrain(
    wpm,
    minimumWPM,
    maximumWPM
);

speakerVolume = constrain(
    speakerVolume,
    minimumVolume,
    maximumVolume
);

sidetoneFrequency = constrain(
    sidetoneFrequency,
    minimumTone,
    maximumTone
);


updateWpmTiming();
updateCannedMessages();


}

// ============================================================
// Save settings
// ============================================================

void saveSettings()
{
preferences.begin("morseduino", false);


preferences.putInt(
    "wpm",
    wpm
);

preferences.putInt(
    "volume",
    speakerVolume
);

preferences.putInt(
    "tone",
    sidetoneFrequency
);

preferences.putInt(
    "rxMode",
    static_cast<int>(rxFrequencyMode)
);

preferences.putInt(
    "rxFreq",
    manualRxFrequency
);

preferences.putInt(
    "audio",
    static_cast<int>(audioSource)
);

preferences.putString(
    "callsign",
    callsign
);

preferences.end();

M5Cardputer.Speaker.setVolume(
    speakerVolume
);


}

// ============================================================
// Radio key DOWN
// ============================================================

void keyDown()
{
digitalWrite(
keyPin,
HIGH
);


M5Cardputer.Speaker.tone(
    sidetoneFrequency
);


}

// ============================================================
// Radio key UP
// ============================================================

void keyUp()
{
digitalWrite(
keyPin,
LOW
);


M5Cardputer.Speaker.stop();


}

// ============================================================
// Transmit Morse
// ============================================================

void transmitMorse(
const char* morse
)
{
while (*morse)
{
if (*morse == '.')
{
keyDown();


        delay(dotDuration);

        keyUp();

        delay(elementSpace);
    }
    else if (*morse == '-')
    {
        keyDown();

        delay(dashDuration);

        keyUp();

        delay(elementSpace);
    }

    morse++;
}

delay(letterSpace);


}

// ============================================================
// Clear text
// ============================================================

void clearText()
{
txBuffer = "";

updateDisplay();


}

// ============================================================
// Main display
// ============================================================

void updateDisplay()
{
M5Cardputer.Display.fillScreen(BLACK);


// --------------------------------------------------------
// Header
// --------------------------------------------------------

M5Cardputer.Display.setTextSize(2);

M5Cardputer.Display.setTextColor(
    GREEN
);

M5Cardputer.Display.setCursor(
    2,
    2
);

M5Cardputer.Display.print(
    "MORSEDUINO"
);


// WPM

M5Cardputer.Display.setTextColor(
    YELLOW
);

M5Cardputer.Display.setCursor(
    150,
    2
);

M5Cardputer.Display.print(
    wpm
);

M5Cardputer.Display.print(
    " WPM"
);


// --------------------------------------------------------
// Separator
// --------------------------------------------------------

M5Cardputer.Display.setTextColor(
    DARKGREY
);

M5Cardputer.Display.drawLine(
    0,
    15,
    M5Cardputer.Display.width(),
    15
);


// --------------------------------------------------------
// Text
// --------------------------------------------------------

M5Cardputer.Display.setTextColor(
    WHITE
);

M5Cardputer.Display.setCursor(
    2,
    22
);


String displayText = txBuffer;


// Only show the latest section

if (displayText.length() > 150)
{
    displayText = displayText.substring(
        displayText.length() - 150
    );
}


const int charactersPerLine = 39;


for (
    int i = 0;
    i < displayText.length();
    i++
)
{
    M5Cardputer.Display.print(
        displayText[i]
    );

    if (
        (i + 1) %
        charactersPerLine == 0
    )
    {
        M5Cardputer.Display.println();
    }
}


// --------------------------------------------------------
// Footer
// --------------------------------------------------------

M5Cardputer.Display.setTextColor(
    CYAN
);

M5Cardputer.Display.setCursor(
    2,
    115
);

M5Cardputer.Display.print(
    "[/]spd ENT RX"
);

M5Cardputer.Display.setCursor(
    145,
    115
);

M5Cardputer.Display.print(
    "TAB menu"
);


}

// ============================================================
// Receive / decoder display
// ============================================================

void updateReceiveDisplay()
{
    // Draw the static RX screen once.  During decoding we update
    // only the regions that change, which avoids full-screen flicker.
    M5Cardputer.Display.fillScreen(BLACK);

    M5Cardputer.Display.setTextSize(2);
    M5Cardputer.Display.setTextColor(GREEN);
    M5Cardputer.Display.setCursor(2, 2);
    M5Cardputer.Display.print("CW RECEIVE");

    M5Cardputer.Display.setTextColor(YELLOW);
    M5Cardputer.Display.setCursor(150, 2);
    M5Cardputer.Display.print(
        audioSource == AUDIO_MIC ? "MIC" : "EXT"
    );

    M5Cardputer.Display.drawLine(
        0, 18,
        M5Cardputer.Display.width(), 18,
        DARKGREY
    );

    M5Cardputer.Display.setTextSize(1);

    M5Cardputer.Display.setTextColor(CYAN);
    M5Cardputer.Display.setCursor(2, 100);
    M5Cardputer.Display.print("Code:");

    M5Cardputer.Display.setCursor(145, 100);
    M5Cardputer.Display.print("Tone:");

    M5Cardputer.Display.setTextColor(YELLOW);
    M5Cardputer.Display.setCursor(2, 111);
    M5Cardputer.Display.print("RX:");
    M5Cardputer.Display.print(rxToneFrequency);
    M5Cardputer.Display.print("Hz ");
    M5Cardputer.Display.print(
        rxFrequencyMode == RX_FREQ_AUTO ? "AUTO" : "MAN"
    );

    if (audioSource == AUDIO_EXTERNAL && !externalAudioReady)
    {
        M5Cardputer.Display.setTextColor(YELLOW);
        M5Cardputer.Display.setCursor(120, 111);
        M5Cardputer.Display.print("ADC N/A");
    }

    M5Cardputer.Display.setTextColor(CYAN);
    M5Cardputer.Display.setCursor(2, 123);
    M5Cardputer.Display.print("ESC TX   DEL clear");

    updateReceiveText();
    updateMorseIndicator();
    updateToneIndicator();
}


// ============================================================
// Update decoded receive text only
// ============================================================

void updateReceiveText()
{
    const int charactersPerLine = 18;
    const int visibleLines = 3;
    const int maxVisibleCharacters =
        charactersPerLine * visibleLines;

    // Clear the whole RX text area first.
    M5Cardputer.Display.fillRect(
        0,
        20,
        M5Cardputer.Display.width(),
        76,
        BLACK
    );

    M5Cardputer.Display.setTextSize(2);
    M5Cardputer.Display.setTextColor(WHITE);
    M5Cardputer.Display.setCursor(2, 25);

    String displayText = rxText;

    // Once the screen is full, show only the newest text.
    if (displayText.length() > maxVisibleCharacters)
    {
        displayText = displayText.substring(
            displayText.length() - maxVisibleCharacters
        );
    }

    for (int i = 0; i < displayText.length(); i++)
    {
        M5Cardputer.Display.print(displayText[i]);

        if ((i + 1) % charactersPerLine == 0)
        {
            M5Cardputer.Display.println();
        }
    }
}

// ============================================================
// Update current dot/dash sequence only
// ============================================================

void updateMorseIndicator()
{
    M5Cardputer.Display.fillRect(
        34, 96,
        105, 14,
        BLACK
    );

    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setTextColor(CYAN);
    M5Cardputer.Display.setCursor(34, 100);
    M5Cardputer.Display.print(rxMorseBuffer);
}


// ============================================================
// Update tone indicator only
// ============================================================

void updateToneIndicator()
{
    M5Cardputer.Display.fillRect(
        180, 96,
        58, 14,
        BLACK
    );

    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setTextColor(CYAN);
    M5Cardputer.Display.setCursor(180, 100);

    if (decoderToneState)
    {
        M5Cardputer.Display.print("ON");
    }
    else
    {
        M5Cardputer.Display.print("OFF");
    }
}


// ============================================================
// Enter receive mode
// ============================================================

void enterReceiveMode()
{
    rxMorseBuffer = "";

    decoderToneState = false;
    decoderCandidateState = false;
    decoderCandidateCount = 0;

    decoderStateStart = millis();
    decoderSilenceStart = millis();
    decoderWordSpaceAdded = false;

    autoCandidateFrequency = manualRxFrequency;
    autoCandidateCount = 0;
    autoFrequencyLocked = false;
    lastAutoToneSeen = millis();

    if (rxFrequencyMode == RX_FREQ_MANUAL)
    {
        rxToneFrequency = manualRxFrequency;
    }

    // Cardputer microphone and speaker share audio hardware.
    // Stop the sidetone speaker before starting the microphone.
    M5Cardputer.Speaker.stop();
    M5Cardputer.Speaker.end();

    if (audioSource == AUDIO_MIC)
    {
        M5Cardputer.Mic.begin();
    }

    appMode = MODE_RECEIVE;
    updateReceiveDisplay();
}


// ============================================================
// Leave receive mode
// ============================================================

void leaveReceiveMode()
{
    if (M5Cardputer.Mic.isEnabled())
    {
        while (M5Cardputer.Mic.isRecording())
        {
            delay(1);
        }

        M5Cardputer.Mic.end();
    }

    M5Cardputer.Speaker.begin();
    M5Cardputer.Speaker.setVolume(speakerVolume);

    appMode = MODE_TRANSMIT;
    updateDisplay();
}


// ============================================================
// Built-in microphone reader
// ============================================================

bool readMicAudioBlock(int16_t* samples, size_t count)
{
    if (!M5Cardputer.Mic.isEnabled())
        return false;

    return M5Cardputer.Mic.record(
        samples,
        count,
        RX_SAMPLE_RATE
    );
}


// ============================================================
// Future external ADC reader
// ============================================================
//
// Put the I2C ADC sampling code here later.  It should fill
// "samples" with signed PCM-style samples and return true.
// ============================================================

bool readExternalAudioBlock(int16_t* samples, size_t count)
{
    (void)samples;
    (void)count;

    return false;
}


// ============================================================
// Goertzel tone measurement
// ============================================================

float measureToneRatio(
    const int16_t* samples,
    size_t count,
    int frequency
)
{
    if (count == 0)
        return 0.0f;

    float mean = 0.0f;

    for (size_t i = 0; i < count; i++)
    {
        mean += samples[i];
    }

    mean /= static_cast<float>(count);

    const float omega =
        2.0f * PI * static_cast<float>(frequency) /
        static_cast<float>(RX_SAMPLE_RATE);

    const float coefficient =
        2.0f * cosf(omega);

    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;

    float totalEnergy = 0.0f;

    for (size_t i = 0; i < count; i++)
    {
        float sample =
            static_cast<float>(samples[i]) - mean;

        totalEnergy += sample * sample;

        q0 = coefficient * q1 - q2 + sample;
        q2 = q1;
        q1 = q0;
    }

    if (totalEnergy < 1.0f)
        return 0.0f;

    const float goertzelPower =
        q1 * q1 +
        q2 * q2 -
        coefficient * q1 * q2;

    return goertzelPower /
        (totalEnergy * static_cast<float>(count));
}


// ============================================================
// AUTO RX frequency scanner / tracker
// ============================================================

void updateRxFrequencyTracking(
    const int16_t* samples,
    size_t count
)
{
    if (rxFrequencyMode == RX_FREQ_MANUAL)
    {
        rxToneFrequency = manualRxFrequency;
        return;
    }

    int scanLow = autoScanStart;
    int scanHigh = autoScanEnd;
    int scanStep = autoScanStep;

    // Once locked, search only near the current tone.  This
    // prevents another nearby signal or noise peak stealing lock.
    if (autoFrequencyLocked)
    {
        scanLow = max(
            minimumRxFrequency,
            rxToneFrequency - 75
        );

        scanHigh = min(
            maximumRxFrequency,
            rxToneFrequency + 75
        );
    }

    float bestRatio = 0.0f;
    int bestFrequency = rxToneFrequency;

    for (int frequency = scanLow;
         frequency <= scanHigh;
         frequency += scanStep)
    {
        float ratio =
            measureToneRatio(
                samples,
                count,
                frequency
            );

        if (ratio > bestRatio)
        {
            bestRatio = ratio;
            bestFrequency = frequency;
        }
    }

    unsigned long now = millis();

    if (bestRatio >= autoFrequencyThreshold)
    {
        lastAutoToneSeen = now;

        // Require a candidate to persist for several blocks before
        // changing frequency, otherwise the display/decoder would
        // chase short noise peaks.
        if (abs(bestFrequency - autoCandidateFrequency)
            <= autoScanStep)
        {
            if (autoCandidateCount < 255)
                autoCandidateCount++;
        }
        else
        {
            autoCandidateFrequency = bestFrequency;
            autoCandidateCount = 1;
        }

        if (autoCandidateCount >= autoCandidateRequired)
        {
            bool frequencyChanged =
                rxToneFrequency != autoCandidateFrequency;

            rxToneFrequency = autoCandidateFrequency;
            autoFrequencyLocked = true;

            if (frequencyChanged)
            {
                // Refresh the small frequency/status line.
                M5Cardputer.Display.fillRect(
                    0, 109,
                    115, 12,
                    BLACK
                );

                M5Cardputer.Display.setTextSize(1);
                M5Cardputer.Display.setTextColor(YELLOW);
                M5Cardputer.Display.setCursor(2, 111);
                M5Cardputer.Display.print("RX:");
                M5Cardputer.Display.print(rxToneFrequency);
                M5Cardputer.Display.print("Hz AUTO");
            }
        }
    }
    else
    {
        autoCandidateCount = 0;

        if (autoFrequencyLocked &&
            now - lastAutoToneSeen >= autoUnlockDelay)
        {
            autoFrequencyLocked = false;
        }
    }
}


// ============================================================
// CW tone detector
// ============================================================

bool detectCWTone(
    const int16_t* samples,
    size_t count
)
{
    const float toneRatio =
        measureToneRatio(
            samples,
            count,
            rxToneFrequency
        );

    return toneRatio > decoderToneThreshold;
}


// ============================================================
// Reverse Morse lookup
// ============================================================

char decodeMorse(const String& code)
{
    for (int i = 0; i < 40; i++)
    {
        if (code == morseCodeMap[i])
        {
            if (i < 26)
                return static_cast<char>('A' + i);

            if (i < 36)
                return static_cast<char>('0' + (i - 26));

            switch (i)
            {
                case 36: return '/';
                case 37: return '?';
                case 38: return '=';
                case 39: return '.';
            }
        }
    }

    return '?';
}


// ============================================================
// Finish one Morse character
// ============================================================

void commitDecodedCharacter()
{
    if (rxMorseBuffer.length() == 0)
        return;

    rxText += decodeMorse(rxMorseBuffer);
    rxMorseBuffer = "";

    if (rxText.length() > 300)
    {
        rxText = rxText.substring(
            rxText.length() - 300
        );
    }

    updateReceiveText();
    updateMorseIndicator();
}


// ============================================================
// CW decoder
// ============================================================

void processDecoder()
{
    bool blockReady = false;

    if (audioSource == AUDIO_MIC)
    {
        blockReady = readMicAudioBlock(
            rxAudioBlock,
            RX_BLOCK_SAMPLES
        );
    }
    else
    {
        blockReady = readExternalAudioBlock(
            rxAudioBlock,
            RX_BLOCK_SAMPLES
        );
    }

    if (!blockReady)
        return;

    updateRxFrequencyTracking(
        rxAudioBlock,
        RX_BLOCK_SAMPLES
    );

    const bool rawTone = detectCWTone(
        rxAudioBlock,
        RX_BLOCK_SAMPLES
    );

    // Require two matching 10 ms blocks before changing state.
    if (rawTone == decoderCandidateState)
    {
        if (decoderCandidateCount < 255)
            decoderCandidateCount++;
    }
    else
    {
        decoderCandidateState = rawTone;
        decoderCandidateCount = 1;
    }

    if (decoderCandidateCount >= 2 &&
        decoderCandidateState != decoderToneState)
    {
        const unsigned long now = millis();
        const unsigned long previousDuration =
            now - decoderStateStart;

        // A tone has just ended: classify it.
        if (decoderToneState)
        {
            if (previousDuration <
                static_cast<unsigned long>(dotDuration * 2))
            {
                rxMorseBuffer += '.';
            }
            else
            {
                rxMorseBuffer += '-';
            }

            decoderSilenceStart = now;
            decoderWordSpaceAdded = false;
        }

        decoderToneState = decoderCandidateState;
        decoderStateStart = now;

        updateMorseIndicator();
        updateToneIndicator();
    }

    const unsigned long now = millis();

    if (!decoderToneState)
    {
        const unsigned long silence =
            now - decoderSilenceStart;

        // Character gap: nominally 3 dot units.
        if (rxMorseBuffer.length() > 0 &&
            silence >= static_cast<unsigned long>(
                (dotDuration * 5) / 2
            ))
        {
            commitDecodedCharacter();
        }

        // Word gap: nominally 7 dot units.
        if (!decoderWordSpaceAdded &&
            rxText.length() > 0 &&
            rxText[rxText.length() - 1] != ' ' &&
            silence >= static_cast<unsigned long>(
                (dotDuration * 13) / 2
            ))
        {
            rxText += ' ';
            decoderWordSpaceAdded = true;
            updateReceiveText();
        }
    }
}


// ============================================================
// Keyboard - receive mode
// ============================================================

void handleReceiveKeyboard()
{
    if (!M5Cardputer.Keyboard.isChange())
        return;

    if (!M5Cardputer.Keyboard.isPressed())
        return;

    Keyboard_Class::KeysState status =
        M5Cardputer.Keyboard.keysState();

    if (status.esc)
    {
        leaveReceiveMode();
        return;
    }

    if (status.del)
    {
        rxText = "";
        rxMorseBuffer = "";
        updateReceiveText();
        updateMorseIndicator();
        return;
    }
}


// ========================================================
// Canned Messages
// ========================================================

void updateCannedDisplay()
{
    M5Cardputer.Display.fillScreen(BLACK);
    M5Cardputer.Display.setTextSize(2);
    M5Cardputer.Display.setTextColor(GREEN);
    M5Cardputer.Display.setCursor(2,2);
    M5Cardputer.Display.println("CANNED MSGS");

    for(int i = 0; i < cannedMessageCount; i++)
    {
        if(i == cannedSelection)
        {
            M5Cardputer.Display.setTextColor(BLACK, WHITE);
        }
        else
        {
            M5Cardputer.Display.setTextColor(WHITE, BLACK);
        }

        M5Cardputer.Display.setCursor(5, 25 + (i * 15));
        M5Cardputer.Display.print(i + 1);
        M5Cardputer.Display.print(" ");
        M5Cardputer.Display.print(cannedMessages[i]);
    }

    M5Cardputer.Display.setTextColor(CYAN);
    M5Cardputer.Display.setCursor(2,115);
    M5Cardputer.Display.print("UP/DN SEL ENT LOAD");
};

// ============================================================
// Audio input setting
// ============================================================

void updateAudioInDisplay()
{
    M5Cardputer.Display.fillScreen(BLACK);

    M5Cardputer.Display.setTextSize(2);
    M5Cardputer.Display.setTextColor(GREEN);
    M5Cardputer.Display.setCursor(2, 2);
    M5Cardputer.Display.print("AUDIO INPUT");

    M5Cardputer.Display.drawLine(
        0,
        18,
        M5Cardputer.Display.width(),
        18
    );

    const char* labels[2] =
    {
        "Internal MIC",
        "External ADC"
    };

    const int firstY = 30;
    const int rowHeight = 25;

    for (int i = 0; i < 2; i++)
    {
        int y = firstY + (i * rowHeight);

        if (i == audioSelection)
        {
            M5Cardputer.Display.setTextColor(BLACK, WHITE);
        }
        else
        {
            M5Cardputer.Display.setTextColor(WHITE, BLACK);
        }

        M5Cardputer.Display.setCursor(5, y);
        M5Cardputer.Display.print(labels[i]);
    }

    M5Cardputer.Display.setTextColor(CYAN, BLACK);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setCursor(2, 108);
    M5Cardputer.Display.print("UP/DN select  ENTER accept");

    M5Cardputer.Display.setCursor(2, 122);
    M5Cardputer.Display.print("ESC cancel");
}


// ============================================================
// Audio input keyboard handler
// ============================================================

void handleAudioInKeyboard()
{
    if (!M5Cardputer.Keyboard.isChange())
    {
        return;
    }

    if (!M5Cardputer.Keyboard.isPressed())
    {
        return;
    }

    Keyboard_Class::KeysState status =
        M5Cardputer.Keyboard.keysState();

    // Cancel and return to settings menu.
    if (status.esc)
    {
        audioSelection = static_cast<int>(audioSource);
        appMode = MODE_MENU;
        updateMenuDisplay();
        return;
    }

    // With only two choices, up/down or left/right simply toggles.
    if (status.up || status.left)
    {
        audioSelection--;

        if (audioSelection < AUDIO_MIC)
        {
            audioSelection = AUDIO_EXTERNAL;
        }

        updateAudioInDisplay();
        return;
    }

    if (status.down || status.right)
    {
        audioSelection++;

        if (audioSelection > AUDIO_EXTERNAL)
        {
            audioSelection = AUDIO_MIC;
        }

        updateAudioInDisplay();
        return;
    }

    // Accept the highlighted input and return to settings.
    if (status.enter)
    {
        audioSource = static_cast<AudioSource>(audioSelection);
        appMode = MODE_MENU;
        updateMenuDisplay();
        return;
    }
}


// ============================================================
// Callsign editor
// ============================================================

void updateCallsignDisplay()
{
    M5Cardputer.Display.fillScreen(BLACK);

    M5Cardputer.Display.setTextSize(2);
    M5Cardputer.Display.setTextColor(GREEN);
    M5Cardputer.Display.setCursor(2, 2);
    M5Cardputer.Display.print("SET CALLSIGN");

    M5Cardputer.Display.drawLine(
        0,
        18,
        M5Cardputer.Display.width(),
        18
    );

    M5Cardputer.Display.setTextColor(WHITE);
    M5Cardputer.Display.setCursor(10, 40);
    M5Cardputer.Display.print(callsignEditBuffer);

    int cursorX = 10 + (callsignEditBuffer.length() * 12);

    if (cursorX > 225)
        cursorX = 225;

    M5Cardputer.Display.drawLine(
        cursorX,
        58,
        cursorX + 8,
        58,
        YELLOW
    );

    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setTextColor(CYAN);
    M5Cardputer.Display.setCursor(2, 98);
    M5Cardputer.Display.print("Type new call to replace current");

    M5Cardputer.Display.setCursor(2, 111);
    M5Cardputer.Display.print("DEL clear/backspace ENTER save");

    M5Cardputer.Display.setCursor(2, 123);
    M5Cardputer.Display.print("ESC cancel");
}


void handleCallsignKeyboard()
{
    if (!M5Cardputer.Keyboard.isChange())
        return;

    if (!M5Cardputer.Keyboard.isPressed())
        return;

    Keyboard_Class::KeysState status =
        M5Cardputer.Keyboard.keysState();

    if (status.esc)
    {
        callsignEditBuffer = callsign;
        callsignReplaceOnType = true;
        appMode = MODE_MENU;
        updateMenuDisplay();
        return;
    }

    if (status.del)
    {
        if (callsignReplaceOnType)
        {
            callsignEditBuffer = "";
            callsignReplaceOnType = false;
        }
        else if (callsignEditBuffer.length() > 0)
        {
            callsignEditBuffer.remove(
                callsignEditBuffer.length() - 1
            );
        }

        updateCallsignDisplay();
        return;
    }

    if (status.enter)
    {
        callsignEditBuffer.trim();
        callsignEditBuffer.toUpperCase();

        if (callsignEditBuffer.length() == 0)
        {
            callsignEditBuffer = "MYCALL";
        }

        callsign = callsignEditBuffer;
        callsignReplaceOnType = true;

        updateCannedMessages();

        preferences.begin("morseduino", false);
        preferences.putString("callsign", callsign);
        preferences.end();

        appMode = MODE_MENU;
        updateMenuDisplay();
        return;
    }

    for (auto c : status.word)
    {
        c = toupper((unsigned char)c);

        bool validCharacter =
            (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') ||
            c == '/';

        if (validCharacter)
        {
            if (callsignReplaceOnType)
            {
                callsignEditBuffer = "";
                callsignReplaceOnType = false;
            }

            if (callsignEditBuffer.length() < maximumCallsignLength)
            {
                callsignEditBuffer += c;
                updateCallsignDisplay();
            }
        }
    }
}


// ============================================================
// Menu display
// ============================================================

void updateMenuDisplay()
{
M5Cardputer.Display.fillScreen(
BLACK
);


// --------------------------------------------------------
// Title
// --------------------------------------------------------

M5Cardputer.Display.setTextColor(
    GREEN
);

M5Cardputer.Display.setTextSize(1);

M5Cardputer.Display.setCursor(
    2,
    2
);

M5Cardputer.Display.print(
    "MORSEDUINO SETTINGS"
);


M5Cardputer.Display.drawLine(
    0,
    15,
    M5Cardputer.Display.width(),
    15
);

// --------------------------------------------------------
// Menu items
// --------------------------------------------------------

const int firstY = 19;
const int rowHeight = 12;


for (
    int i = 0;
    i < MENU_COUNT;
    i++
)
{
    int y =
        firstY +
        (i * rowHeight);


    // Selected item

    if (i == menuSelection)
    {
        M5Cardputer.Display.setTextColor(
            BLACK,
            WHITE
        );
    }
    else
    {
        M5Cardputer.Display.setTextColor(
            WHITE,
            BLACK
        );
    }


    M5Cardputer.Display.setCursor(
        5,
        y
    );


    switch (i)
    {
        case MENU_WPM:

            M5Cardputer.Display.print(
                "WPM"
            );

            M5Cardputer.Display.setCursor(
                150,
                y
            );

            M5Cardputer.Display.print(
                wpm
            );

            break;


        case MENU_VOLUME:

            M5Cardputer.Display.print(
                "Volume"
            );

            M5Cardputer.Display.setCursor(
                150,
                y
            );

            M5Cardputer.Display.print(
                speakerVolume
            );

            break;


        case MENU_TONE:

            M5Cardputer.Display.print(
                "TX Tone"
            );

            M5Cardputer.Display.setCursor(
                150,
                y
            );

            M5Cardputer.Display.print(
                sidetoneFrequency
            );

            M5Cardputer.Display.print(
                " Hz"
            );

            break;

        case MENU_RX_MODE:

            M5Cardputer.Display.print(
                "RX Mode"
            );

            M5Cardputer.Display.setCursor(
                150,
                y
            );

            M5Cardputer.Display.print(
                rxFrequencyMode == RX_FREQ_AUTO ?
                "AUTO" : "MANUAL"
            );

            break;

        case MENU_RX_FREQUENCY:

            M5Cardputer.Display.print(
                "RX Freq"
            );

            M5Cardputer.Display.setCursor(
                150,
                y
            );

            if (rxFrequencyMode == RX_FREQ_AUTO)
            {
                M5Cardputer.Display.print("AUTO");
            }
            else
            {
                M5Cardputer.Display.print(
                    manualRxFrequency
                );

                M5Cardputer.Display.print(
                    " Hz"
                );
            }

            break;

        case MENU_AUDIO_IN:

            M5Cardputer.Display.print(
                "Set Audio Input"
            );

            break;

        case MENU_CALLSIGN:

            M5Cardputer.Display.print(
                "Callsign"
            );

            M5Cardputer.Display.setCursor(
                150,
                y
            );

            M5Cardputer.Display.print(
                callsign
            );

            break;

        case MENU_CLEAR:

            M5Cardputer.Display.print(
                "Clear text"
            );

            break;


        case MENU_SAVE:

            M5Cardputer.Display.print(
                "Save settings"
            );

            break;
    }
}


// Reset colours

M5Cardputer.Display.setTextColor(
    WHITE,
    BLACK
);


// --------------------------------------------------------
// Footer
// --------------------------------------------------------

M5Cardputer.Display.setTextSize(1);

M5Cardputer.Display.setCursor(
    2,
    122
);

M5Cardputer.Display.setTextColor(
    CYAN
);

M5Cardputer.Display.print(
    "UP/DN Sel  LT/RT Change"
);


}

// ============================================================
// Change selected menu value
// ============================================================

void changeMenuValue(
int amount
)
{
switch (menuSelection)
{
case MENU_WPM:


        wpm += amount;

        wpm = constrain(
            wpm,
            minimumWPM,
            maximumWPM
        );

        updateWpmTiming();

        break;


    case MENU_VOLUME:

        speakerVolume += amount * 5;

        speakerVolume = constrain(
            speakerVolume,
            minimumVolume,
            maximumVolume
        );

        M5Cardputer.Speaker.setVolume(
            speakerVolume
        );

        break;


    case MENU_TONE:

        sidetoneFrequency +=
            amount * toneStep;

        sidetoneFrequency = constrain(
            sidetoneFrequency,
            minimumTone,
            maximumTone
        );

        break;


    case MENU_RX_MODE:

        if (amount != 0)
        {
            rxFrequencyMode =
                (rxFrequencyMode == RX_FREQ_AUTO) ?
                RX_FREQ_MANUAL :
                RX_FREQ_AUTO;

            if (rxFrequencyMode == RX_FREQ_MANUAL)
            {
                rxToneFrequency = manualRxFrequency;
            }
        }

        break;


    case MENU_RX_FREQUENCY:

        if (rxFrequencyMode == RX_FREQ_MANUAL)
        {
            manualRxFrequency +=
                amount * rxFrequencyStep;

            manualRxFrequency = constrain(
                manualRxFrequency,
                minimumRxFrequency,
                maximumRxFrequency
            );

            rxToneFrequency = manualRxFrequency;
        }

        break;


    default:

        break;
}


updateMenuDisplay();


}

// ============================================================
// Select menu item
// ============================================================

void selectMenuItem()
{
switch (menuSelection)
{
case MENU_CLEAR:


        clearText();

        appMode = MODE_TRANSMIT;

        break;

    
    case MENU_AUDIO_IN:

        audioSelection = static_cast<int>(audioSource);
        appMode = MODE_AUDIO_IN;
        updateAudioInDisplay();

        break;


    case MENU_CALLSIGN:

        callsignEditBuffer = callsign;
        callsignReplaceOnType = true;
        appMode = MODE_CALLSIGN;
        updateCallsignDisplay();

        break;


    case MENU_SAVE:

        saveSettings();

        appMode = MODE_TRANSMIT;

        break;


    default:

        break;
}


if (appMode == MODE_TRANSMIT)
{
    updateDisplay();
}


}

// ============================================================
// Process normal keyboard character
// ============================================================

void processTransmitCharacter(
char c
)
{
// --------------------------------------------------------
// WPM down
// --------------------------------------------------------


if (c == '[')
{
    wpm--;

    wpm = constrain(
        wpm,
        minimumWPM,
        maximumWPM
    );

    updateWpmTiming();
    updateDisplay();

    return;
}


// --------------------------------------------------------
// WPM up
// --------------------------------------------------------

if (c == ']')
{
    wpm++;

    wpm = constrain(
        wpm,
        minimumWPM,
        maximumWPM
    );

    updateWpmTiming();
    updateDisplay();

    return;
}


// --------------------------------------------------------
// Space
// --------------------------------------------------------

if (c == ' ')
{
    txBuffer += ' ';

    updateDisplay();


    // transmitMorse() already creates
    // 3 units of character spacing.
    //
    // Add another 4 units to make
    // the total word gap 7 units.

    delay(
        wordSpace -
        letterSpace
    );

    return;
}



// --------------------------------------------------------
// Convert to upper case
// --------------------------------------------------------

c = toupper(
    (unsigned char)c
);


// --------------------------------------------------------
// Morse lookup
// --------------------------------------------------------

const char* morse =
    getMorseCode(c);


if (morse[0] == '\0')
{
    return;
}


// --------------------------------------------------------
// Display
// --------------------------------------------------------

txBuffer += c;

updateDisplay();


// --------------------------------------------------------
// Transmit
// --------------------------------------------------------

transmitMorse(
    morse
);


}

// ============================================================
// Keyboard - transmit mode
// ============================================================

void handleTransmitKeyboard()
{
if (
!M5Cardputer.Keyboard.isChange()
)
{
return;
}


if (
    !M5Cardputer.Keyboard.isPressed()
)
{
    return;
}


Keyboard_Class::KeysState status =
    M5Cardputer.Keyboard.keysState();


// --------------------------------------------------------
// Function keys
// --------------------------------------------------------

if (status.tab)
{
    appMode = MODE_MENU;

    menuSelection = MENU_WPM;

    updateMenuDisplay();

    return;
}

// --------------------------------------------------------
// Receive / decoder mode
// --------------------------------------------------------

if (status.enter)
{
    enterReceiveMode();
    return;
}

// -------------------------------------------------------
//  clear the screen
// -------------------------------------------------------

if (status.del)
{
    if (txBuffer.length() > 0)
    {
        txBuffer.remove(
            txBuffer.length() - 1
        );

        updateDisplay();
    }

    return;
}

// --------------------------------------------------------
// canned messages
// --------------------------------------------------------
if (status.esc)
{
    appMode = MODE_CANNED;
    cannedSelection = 0;
    updateCannedDisplay();
    return;
}


// --------------------------------------------------------
// Normal characters
// --------------------------------------------------------

for (
    auto c : status.word
)
{
    processTransmitCharacter(
        c
    );
}

}

// ============================================================
// Keyboard - menu mode
// ============================================================

void handleMenuKeyboard()
{
if (
!M5Cardputer.Keyboard.isChange()
)
{
return;
}


if (
    !M5Cardputer.Keyboard.isPressed()
)
{
    return;
}


Keyboard_Class::KeysState status =
    M5Cardputer.Keyboard.keysState();


// --------------------------------------------------------
// Escape
// --------------------------------------------------------

if (status.esc)
{
    appMode = MODE_TRANSMIT;

    updateDisplay();

    return;
}


// --------------------------------------------------------
// Up
// --------------------------------------------------------

if (status.del)
{

    // clears text without going into menu
}


// --------------------------------------------------------
// Navigation through raw key codes
// --------------------------------------------------------

if (status.word.size() > 0)
{
    for (auto c : status.word)
    {
        switch (c)
        {
            case '\x1b':

                appMode = MODE_TRANSMIT;

                updateDisplay();

                return;


            default:

                break;
        }
    }
}


// --------------------------------------------------------
// Cursor keys
//
// Cardputer keyboard library reports these through
// the special key state fields on supported versions.
// --------------------------------------------------------

if (status.up)
{
    menuSelection--;

    if (menuSelection < 0)
    {
        menuSelection =
            MENU_COUNT - 1;
    }

    updateMenuDisplay();
}


if (status.down)
{
    menuSelection++;

    if (menuSelection >= MENU_COUNT)
    {
        menuSelection = 0;
    }

    updateMenuDisplay();
}


if (status.left)
{
    changeMenuValue(-1);
}


if (status.right)
{
    changeMenuValue(1);
}


if (status.enter)
{
    selectMenuItem();
}


}

// ============================================================
// Canned Messages
// ============================================================

void handleCannedKeyboard()
{
    if (!M5Cardputer.Keyboard.isChange()) return;
    if (!M5Cardputer.Keyboard.isPressed()) return;

    auto status = M5Cardputer.Keyboard.keysState();

    if (status.esc)
    {
        appMode = MODE_TRANSMIT;
        updateDisplay();
        return;
    }

    if (status.up)
    {
        cannedSelection--;

        if (cannedSelection < 0)
            cannedSelection = cannedMessageCount - 1;

        updateCannedDisplay();
    }

    if (status.down)
    {
        cannedSelection++;

        if (cannedSelection >= cannedMessageCount)
            cannedSelection = 0;

        updateCannedDisplay();
    }

    if (status.enter)
    {
        txBuffer = cannedMessages[cannedSelection];

        updateDisplay();

    for (int i = 0; i < txBuffer.length(); i++)
    {
        char c = toupper(txBuffer[i]);

        if (c == ' ')
        {
            delay(wordSpace);
            continue;
        }

        const char* morse = getMorseCode(c);

        if (morse[0] != '\0')
        {
            transmitMorse(morse);
        }
    }

    appMode = MODE_TRANSMIT;
    updateDisplay();
    clearText();     //clear screen after Tx
    return;
    }
}

// ============================================================
// Setup
// ============================================================

void setup()
{
auto cfg = M5.config();


M5Cardputer.begin(
    cfg,
    true
);


// --------------------------------------------------------
// Radio key output
// --------------------------------------------------------

pinMode(
    keyPin,
    OUTPUT
);


// Radio MUST start unkeyed

digitalWrite(
    keyPin,
    LOW
);


// --------------------------------------------------------
// Serial
// --------------------------------------------------------

Serial.begin(
    115200
);


delay(200);


// --------------------------------------------------------
// Load saved settings
// --------------------------------------------------------

loadSettings();


// --------------------------------------------------------
// Speaker
// --------------------------------------------------------

M5Cardputer.Speaker.setVolume(
    speakerVolume
);


// --------------------------------------------------------
// Display
// --------------------------------------------------------

M5Cardputer.Display.setRotation(
    1
);


updateDisplay();


// --------------------------------------------------------
// Startup message
// --------------------------------------------------------

Serial.println();
Serial.println(
    "=========================="
);

Serial.println(
    "Morseduino Cardputer ADV"
);

Serial.println(
    "=========================="
);

Serial.print(
    "WPM: "
);

Serial.println(
    wpm
);

Serial.print(
    "Volume: "
);

Serial.println(
    speakerVolume
);

Serial.print(
    "Tone: "
);

Serial.println(
    sidetoneFrequency
);

Serial.print(
    "RX mode: "
);

Serial.println(
    rxFrequencyMode == RX_FREQ_AUTO ?
    "AUTO" : "MANUAL"
);

Serial.print(
    "Manual RX frequency: "
);

Serial.print(
    manualRxFrequency
);

Serial.println(
    " Hz"
);

Serial.print(
    "Callsign: "
);

Serial.println(
    callsign
);

Serial.println(
    "Key output: G1"
);

Serial.println();


}

// ============================================================
// Main loop
// ============================================================

void loop()
{
M5Cardputer.update();


switch(appMode)
{
    case MODE_TRANSMIT:
        handleTransmitKeyboard();
        break;

    case MODE_MENU:
        handleMenuKeyboard();
        break;

    case MODE_CANNED:
        handleCannedKeyboard();
        break;

    case MODE_AUDIO_IN:
        handleAudioInKeyboard();
        break;

    case MODE_CALLSIGN:
        handleCallsignKeyboard();
        break;

    case MODE_RECEIVE:
        processDecoder();
        handleReceiveKeyboard();
        break;
}


}