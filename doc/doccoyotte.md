Coyotte user manual
===================

Plugging
--------

Several components have to be plugged before use:

* Plug first the USB cable on the computer (any port)
* Launch Cygwin (icon on the desktop) and type in~: \textsf{screen /dev/tty.S4} (S4 can be S5 depending on the port you plugged the USB cable in). You should then see several columns of numbers, one of them is the current temperature.
* Plug the temperature sensor (purple cable), this is not polarized and hence corresponds to the two wires of identical color.
* Plug the Peltier module (green and purple wires).
* Plug the +12V power adaptor

Keyboard control
----------------

To control the temperature, you have to use the keyboard:

\begin{tabular}{llll}
   Key & Function\\
   \hline
   ``p'' & ON/OFF\\
   ``+/-'' & 0.5 target temperature increment\\
   ``a'' & Target=18\degre C\\
   ``b'' & Target=20\degre C\\
   ``c'' & Target=21\degre C\\
   ``d'' & Target=25\degre C\\
   ``e'' & Target=29\degre C\\
\end{tabular}

Advanced control
----------------

The PID parameters can be tuned in real time:

\begin{tabular}{llll}
   Key & Function\\
   \hline
   ``u/j'' & P increment\\
   ``i/k'' & I increment\\
   ``o/l'' & D increment\\
\end{tabular}


Author
------

Hervé Rouault <herve.rouault@pasteur.fr>


