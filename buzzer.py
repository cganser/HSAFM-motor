#!/home/afm/python/daq_venv/bin/python

#Passive buzzer control
#
#Manual list of frequencies and times
#freq = [1000,0,1000,0,500,0,0,0,0,0,0]
#time = [0.1,0.001,0.1,0.001,0.1,0.01,0.2,0.01,0.1,0.01,0.1]
#buzzer.setMelody(freq,time)


#Note input with same duration and pause length
#time = 0.1
#pause = 0.05

#buzzer.setMelody2("C5 C5 C5 C5 Ab4 C5 Bb4 C5",time,pause)
#buzzer.playMelody()



import RPi.GPIO as GPIO

from time import sleep

class ApproachBuzzer():
    def __init__(self,pin=11,buzzer="passive"):
        self.pin = pin

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin,GPIO.OUT)

        if buzzer == "passive":
            self.buzzerType = 0
            self.freq = 100
            self.dc = 50

            self.fList = []
            self.tList = []

            finished = 0

            while finished == 0:
                try:
                    self.pwm = GPIO.PWM(self.pin,self.freq)
                    finished = 1
                except RuntimeError:
                    sleep(0.1)
                    pass

            self.pwm.start(0)
        else:
            self.buzzerType = 1


    def noteToFreq(self,note,octave):
        f0 = 440

        if note == 'C':
            n=-9
        elif note == 'Db':
            n=-8
        elif note == 'D':
            n=-7
        elif note == 'Eb':
            n=-6
        elif note == 'E':
            n=-5
        elif note == 'F':
            n=-4
        elif note == 'Gb':
            n=-3
        elif note == 'G':
            n=-2
        elif note == 'Ab':
            n=-1
        elif note == 'A':
            n = 0
        elif note == 'Bb':
            n = 1
        elif note == 'B':
            n = 2
        else:
            n = 0

        freq = f0*2**(n/12-4+octave)
        return freq

    def playSound(self,freq,time):
        if self.buzzerType == 0:
            self.pwm.ChangeFrequency(freq)
            self.pwm.ChangeDutyCycle(self.dc)
            sleep(time)
            self.pwm.start(0)
        else:
            GPIO.output(self.pin, GPIO.HIGH)
            sleep(time)
            GPIO.output(self.pin, GPIO.LOW)


    def setMelody(self,freq_list,time_list):
        self.fList = freq_list
        self.tList = time_list

    def setMelody2(self,notes,time,pause):
        #the notes should be put in a string like "A2 Bb D4"
        #the time is the length of each note
        #the pause is the silent period between two notes
        self.fList = []
        self.tList = []

        sep_notes = notes.split(' ')

        for item in sep_notes:
            note = item[0:len(item)-1]
            octave = int(item[len(item)-1])
            freq = self.noteToFreq(note,octave)

            self.fList.append(freq)
            self.tList.append(time)
            self.fList.append(0)
            self.tList.append(pause)

    def playMelody(self):
        for i in range(0,len(self.fList)):
            if self.fList[i] != 0:
                self.playSound(self.fList[i], self.tList[i])
            else:
                sleep(self.tList[i])

    def playStandardSound(self):

        if self.buzzerType == 0:
            startF = 1
            time = 0.01

            for i in range(20,50):
                self.playSound(i**2*startF,time)

            for i in range(20,50):
                self.playSound(i**2*startF,time)

            for i in range(20,55):
                self.playSound(i**2*startF,time)

        else:
            self.playSound(0,2)

    def playStandardSound2(self):

        if self.buzzerType == 0:

            freq = [500, 0, 500, 0, 800]
            time = [0.1, 0.1, 0.1, 0.01, 0.3]
            self.setMelody(freq,time)
            self.playMelody()

        else:
            self.playSound(0,2)

    def sweep(self,df,dt,startN,N):
        for i in range(startN,N):
            f = i*df
            self.playSound(f,dt)


#buz = ApproachBuzzer()
#buz.setMelody2("A4 C5 C5 D4 A5 A5 A5 D4 E4 E5 D4 D5 F4 F5 G4 Ab4 A4",0.15,0.00)
#buz.playMelody()
#buz.sweep(1,0.01,1,5700)
