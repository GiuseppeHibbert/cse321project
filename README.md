<h1>CSE 321: Tracking Turret Sentry Project</h1>
<img src="/images/sentry.jpg"></img>

<h2>Setup</h2>
<li>1. Use the included USB Cable to plug the Sentry into a laptop or computer</li>
<li>2. Download the source code and flash the sentry.ino file into the machine using Arduino IDE</li>
<li>3. Now wait a few moments for the program to initialize into the machine</li>
<li>4. The sentry is now initialized and is ready to use.</li>

<h2>Usage</h2>
<li>1. The turret has three main states, Search, Tracking, and Target</li>
<li>2. When the turret is initially plugged in, it is in the Search state. Here, it is looking for an object (See Image Below)</li>
<img src="/images/states.png"></img>
<li>3. The sentry has two ultrasonic sensors which is how it detects an object. If you move an object into sight of just one sensor, the sentry enters tracking mode, which means the laser will activate and the stepper motor will activate and pan towards that object. The activation of the laser simulates the "firing mechanism"</li>
<li>4. After the sentry motor has panned to the object is has detected, it will enter the target state. This state can only be activated if both the left and right sensor are simultaneously detecting the object.</li>
<li>5. Once the turret has entered the target state, it will shine the laser at the target, simulating a "firing mechanism".</li>

