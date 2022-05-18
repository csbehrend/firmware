Model – 2: more precision, something is still can be improved

We have some model for the damper + spring action:

F=Fx,x,

where x is the displacement of the damper. The model is implemented based on the data from dyno plots and a simple linear model for spring force:

F=Fspringx+Fdamperx=-k\*x+Fdamperx

Fdamperx is evaluated based on the interpolation of dyno plots data which is already digitized.

Now, let’s focus on the geometry of the vehicle and the estimation of the normal force. The car is symmetric, so we’ll try to illustrate everything for one side keeping in mind that the symmetric equations hold for the other side of the car. 

We start by calculating the force equilibrium for the plates indicated in the picture. The summary of measurements is shown below:

Point D – other side of the damper. (Damper is DC) 

![](Aspose.Words.1e66a88c-aae7-47f5-9e65-d80c4cf42135.001.png)![](Aspose.Words.1e66a88c-aae7-47f5-9e65-d80c4cf42135.002.png)

Also, the angle between the vertical axis and the diagonal wheel tube is 42.49deg=α. For this model, we are assuming that this angle is constant. It is shown on the picture at the bottom of the next page. The torque equilibrium equation for the plate:

Fd\*Dd=Fw\*Dw+Fa\*Da     (1)

The quantity directly related to the normal force is Fw as pretty much nothing else acts on the wheel in the vertical direction. The horizontal tubes conneting to the wheel are not yielding any vertical force. Also, the acceleration of the wheel is a lot smaller compared to the normal force, so we assume that it’s equal to 0 for now. For the geometry shown, we will get some estimation of angles ∠OA,Fa,∠OB,Fw,∠OC,Fd, then, estimate the Di values based on the damper displacement and then estimate the force.

The damper length will be modelled as OD=x0+x, where x0is the initial length of the damper.

The step-by-step procedure for updating the geometry is shown below:

NOTE: Convention for labeling angles is as follows: ∠(m, n) is an ngle we need to rotate line m counter clockwise to make it parallel to line n. Also, ∠AOB=∠(AO, OB).

1. cos∠OCD=OC2+CD2-OD22\*OC\*CD
1. sin∠OCD=1-cos2∠OCD (absolute value of sine, essentially)
1. Dd=OC\*sin∠OCD
1. ∠DOC=sin-1{sin∠OCD\*CDDO} (taking absolute value in code for consistency)
1. ∠Fw, OB=∠COB+∠DOC-∠OD, vertical+∠Fw, vertical
1. Calculate sin⁡(∠Fw, OB)
1. Dw=OB\*sin∠ Fw,OB
1. ∠vaerical, AO=∠COA+∠DOC+∠OD,vaerical
1. ∠(OA, Fa)=∠(vertical, Fa)-∠(vertical, AO)
1. Da=OA\*sin∠OA,Fa  *(taking absolute value)*

Later, the superscripts r and l refer to the right and left sides of the car. The displacement directions convention is the same as for the forces of the picture.

The equation for ARB is also pretty simple. Suppose the vertical displacements of the points Al and Ar on the left and right of the car are xal and xar. For now, we assume that the ARB connections are vertical. If this is a bad assumption, we can always sacle our ARB forces by spme oefficient close to 1. The torque acting on the ARB is equal to (Fal+Far)\*s\*(s)=0. Thus, on each of the sides, the forces compensate each other. The angle twist of the bar will be given as:

ϕ=xal-xars

If we adopt torsion coefficient γ and are careful with signs, we can write the following relations for the forces:

Fal=-Far= γxar-xals2

Now, substituting everything to the first equation, we get:

Fwl=Fdl\*Dd-Fal\*DaDw=Fdl\*Dd- γxar-xals2\*DaDw

Also, note that xa=OA\*sin∠vertical, AO-π2. (This is step 11 in our update geometry algorithm, note that we don’t take absolute value here)

Now, dropping the index d (xd=x), and simplifying a bit, we obtain for the left wheel (same for the right wheel).

Nl=Fwlcosα=Fxl,xl\*Dd+γxar-xals2\*DaDwcosα

![](Aspose.Words.1e66a88c-aae7-47f5-9e65-d80c4cf42135.003.png)

The only things that need to be calibrated are zero-length of the spring, K of the spring, and initial length (zero length of the spring). To calculate these, we use 4 sclaes for each wheel. The initial measurements are perfomed with a bare weight of the car and the second measurement is done after adding some eight to the car to move the dampers positions. 

