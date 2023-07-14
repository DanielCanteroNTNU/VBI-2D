% ---- Glossary of all variables in VBI-2D model ----

% Description of all the variables used in the VBI-2D model.
% These variables are either inputs, outputs or both to different functions
% in the model. Note that all variables are grouped in structure variables.

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% -------------------------------------------------------------------------
% ------------------------------- Calc ------------------------------------
% -------------------------------------------------------------------------

% Calc = Structure with calculation variables and options
%     Calc.Profile = Substructure with information about the road profile
%         Calc.Profile.type = Type of profile:
%             0 = Smooth profile
%             1 = Load an existing profile
%             2 = ISO random profile
%             3 = Step profile
%             4 = Ramp profile
%             5 = Sinewave profile
%         Depending on the profile type, additional variables should be defined
%         for Calc.Proifle.type = 1
%             Calc.Profile.Load.file_name = Name of file to load
%         for Calc.Profile.type = 2
%             Calc.Profile.Info = Substructure with information about the ISO class
%                 Calc.Profile.Info.class = ISO class in text format ('A','B',...)
%                 Calc.Profile.Info.Gd_limtis = Array with the Gd limits for each class as indicated in ISO 8608 code
%                 Calc.Profile.Info.Gd = Actual Gd value of the generated profile 
%                 Calc.Profile.Info.PSD_x = Frequency values of the PSD (x-coordinate)
%                 Calc.Profile.Info.PSD_y = Magnitude values of the PSD (y-coordinate)
%             Calc.Profile.Opt.class_var = Flag to consider variation of roughness within the profile class (1 = yes)
%             Calc.Profile.Spatial_frq = Substructure with information about the spatial frequency 
%                 Calc.Profile.Spatial_frq.min = Minimum spatial frequency to consider
%                 Calc.Profile.Spatial_frq.max = Maximum spatial frequency to consider
%                 Calc.Profile.Spatial_frq.ref = Reference spatial frequency as defined in ISO 8606
%         for Calc.Profile.type = 3
%             Calc.Profile.step_loc_x = Step location in x-coordinate system [m]
%             Calc.Profile.step_height = Step height in [m]
%         for Calc.Profile.type = 4
%             Calc.Profile.ramp_loc_x_0 = Ramp start location in x-coordinate system [m]
%             Calc.Profile.ramp_loc_x_end = Ramp end location in x-coordinate system [m]    
%             Calc.Profile.ramp_height = Ramp height in [m]
%         for Calc.Profile.type = 5
%             Calc.Profile.sine_wavelength = Wavelength of the sinusoidal [m]
%             Calc.Profile.sin_Amp = Amplitude of the sinusoidal [m]
%             Calc.Profile.sin_phase = Phase of sinusoidal at x=0 [rad]
%         Calc.Profile.Load = Substructure with information about profile loading from a file
%             Calc.Profile.Load.on = Flag to switch on/off the profile loading (1=loading)
%             Calc.Profile.Load.file_name = String specifying the name of the file to load
%         Calc.Profile.Save = Substructure with information about profile saving to a file
%             Calc.Profile.Save.on = Flag to switch on/off the profile saving (1=saving)
%             Calc.Profile.Save.file_name = String specifying the name of the file to save
%             Calc.Profile.Save.path = Path indicating the directory where to save the file
%         Calc.Profile.dx = Spatial sampling distance on X direction [m]
%         Calc.Profile.L = Total length of generated profile on X direcction [m]
%         Calc.Profile.needed_x0 = x coordinate furthest to the left needed for this event [m]
%         Calc.Profile.needed_x_end = x coordinate furthest to the right needed for this event [m]
%         Calc.Profile.needed_L = Total length of profile needed for this event [m]
%         Calc.Profile.x = Array with x coordinates of the generated profile
%         Calc.Profile.num_x = Number of sample points of the generated profile
%         Calc.Profile.h = Array with elevation (y coordinates) of the generated profile
%         Calc.Profile.Gd = Actual Gd value of the generated profile (0 for all except for Calc.Profile.type = 2)
%     Calc.Solver = Substructure with information about the numerical solver
%         Calc.Solver.min_t_steps_per_second = Minimum time steps per second
%         Calc.Solver.max_accurate_frq = Maximum accurate frequency of the solution
%         Calc.Solver.min_Beam_modes_considered = Number of beam modes to be considered for time step selection
%         Calc.Solver.t_end = Total time to be simulated
%         Calc.Solver.t_steps_per_second = Time steps per second
%         Calc.Solver.t_steps_criteria = Code number of the final adopted step criteria
%         Calc.Solver.t_steps_criteria_text = String with the text of the adopted step criteria
%             1 = Maximum vehicle frequency
%             2 = Defined maximum bridge modes considered
%             3 = Defined minimum steps per second
%             4 = Profile maximum frequency (spatial frequency x Vehicle velocity)
%             5 = User-defined maximum accurate frequency
%         Calc.Solver.t = Array of time steps to simulate
%         Calc.Solver.dt = Sampling period of t (time step)
%         Calc.Solver.num_t = Total number of time steps
%         Calc.Solver.t0_ind_beam = First time step for the first vehicle to enter the beam
%         Calc.Solver.t_end_ind_beam = Last time step for the last vehicle to leave the beam
%         Calc.Solver.num_t_beam = Number of time steps on the beam
%         Calc.Solver.t_beam = Array of time steps while vehicles on the beam
%         Calc.Solver.NewMark = Substructure with information about the Newmark-Beta integration scheme
%             Calc.Solver.NewMark.damp = Selection of NewMark-Beta scheme
%                 0 = No numerical damping
%                 1 = With numerical damping
%             Calc.Solver.NewMark.beta = Constant of the Newmark-Beta scheme
%             Calc.Solver.NewMark.delta = Constant of the Newmark-Beta scheme
%     Calc.Opt = Substructures with information about different options
%         Calc.Opt.verbose = Flag to display detailed information about the current simulation (1=on)
%         Calc.Opt.show_progress_every_s = Display calculation progress every X seconds
%         Calc.Opt.beam_frq = Flag indicating if frequencies should be calculated (1=on)
%         Calc.Opt.beam_modes = Flag indicatiing if modes of vibration should be calculated (1=on)
%         Calc.Opt.veh_frq = Flag to calculate vehicle's frequencies (1=on)
%         Calc.Opt.vehInitSta = Flag to calculate initial static deformation of vehicle (1=on)
%         Calc.Opt.beamInitSta = Flag to calculate initial static deformation of beam  (1=on)
%         Calc.Opt.calc_mode_BM = How to calculate the BM on the beam
%             0 = Calculations done node by node
%             1 = Average results at the node (Default)
%         Calc.Opt.calc_mode_Shear = How to calculate the Shear on the beam
%             0 = Calculations done node by node
%             1 = Average results at the node (Default)
%         Calc.Opt.VBI = Flag indicating to switch on/off the vehicle-bridge interaction (1=on)
%         Calc.Opt.free_vib_seconds = Additional number of seconds in free vibration to include in the simulation
%         Calc.Opt.include_coriolis = Flag to include coriolis effects (1=on)
%         Calc.Opt.linear_eqv_loads = option to perform the equivalent load distribution of point loads to corresponding nodes using:
%             0 = shape function (Default)
%             1 = linear distribution depending on the distance to the nodes
%     Calc.Plot = Substructure indicating what figures to plot at the end of the simulation
%         Calc.Plot.P1_Beam_frq = Plot the distribution of beam frequencies
%         Calc.Plot.P2_Beam_modes = Plot the first n modes of vibration (of Beam)
%             The number of modes to plots is defined by the value of P2_Beam_modes
%         Calc.Plot.P3_VehPos = Plot the vehicle position (velocity and acceleration)
%             A figure is shown with 3 subplots: 
%             1) Load position in time; 
%             2) Load velocity in time; 
%             3) Load acceleration in time
%         Calc.Plot.P4_Profile = Plot the profiles and corresponding 1st derivatives
%         Calc.Plot.Profile_original = Plot the original profile generated/loaded inside function B19
%         Calc.Plot.P5_Beam_U = Contourplot of Beam deformation
%         Calc.Plot.P6_Beam_U_under_Veh = Deformation under the vehicle
%         Calc.Plot.P7_Veh_U_iter = Vehicle total displacement for each iteration (Only FI procedure)
%         Calc.Plot.P8_Veh_Urel_iter = Vehicle relative displacement for each iteration (Only FI procedure)
%         Calc.Plot.P9_Beam_U_under_Veh_iter = Deformation under the vehicle for each iteration (Only FI procedure)
%         Calc.Plot.P10_diff_iter = The difference between solutions (Iteration criteria) (Only FI procedure)
%         Calc.Plot.P11_Beam_U_static = Calculates the Static Deformation of Beam (Due to Interaction force)
%         Calc.Plot.P13_Interaction_Force = Final interaction force
%         Calc.Plot.P14_Interaction_Force_iter = Interaction force for each iteration (Only FI procedure)
%         Calc.Plot.P16_Beam_BM = Contourplot of Beam BM
%         Calc.Plot.P17_Beam_BM_static = Contourplot of Beam Static BM
%         Calc.Plot.P18_Beam_Shear = Contourplot of Beam Shear
%         Calc.Plot.P19_Beam_Shear_static = Contourplot of Beam Static Shear
%         Calc.Plot.P20_Beam_MidSpan_BM = Static and Dynamic BM at mid-span
%         Calc.Plot.P21_Beam_MidSpan_BM_iter = Static and Dynamic BM at mid-span for various iterations (Only FI procedure)
%         Calc.Plot.P22_SSI_num_iterations = Number of iterations for each time step (Only for SSI procedure)
%         Calc.Plot.P23_vehicles_responses = Vehicles DOF responses (Displacement, velocity and acceleration)
%         Calc.Plot.P24_vehicles_wheel_responses = Responses at the wheels of the vehicle
%         Calc.Plot.P25_vehicles_under_responses = Responses under the wheel (Bridge response at the contact point)
%         Calc.Plot.P26_PSD_Veh_A = PSD of vehicle accelerations
%         Calc.Plot.P27_Veh_A = Time histories of vehicle accelerations
%     Calc.Constant = Substructure with constants
%         Calc.Constant.grav = Gravity acceleration [m/s^2] (Default: -9.81)
%         Calc.Constant.tol = Threshold under which two numerical values are considered identical
%     Calc.Proc = Information about the solution procedure to solve the vehicle-bridge interaction
%         Calc.Proc.name = String with the name of the procedure:
%             'Full_Sim_Iter' = Iterative procedure for whole time-history solution (Full simulation iteration)
%             'StepByStep_Iter' = Step-by-Step iteration 
%             'Coupled' = Coupled system solution (Default and recommended)
%         Calc.Proc.code = Code number of the procedure to use:
%             1 = Full iteration (Full_Sim_Iter)
%             2 = Step-by-step iteration (StepByStep_Iter)
%             3 = Coupled solution
%         Calc.Proc.short_name = Abreviation of procedure name
%             1 = FI
%             2 = SSI
%             3 = Coup
%         Calc.Proc.Iter = Substructure with particular information for the iterative procedures
%             Calc.Proc.Iter.max_num = Maximum number of iterations (Default: 10)
%             Calc.Proc.Iter.criteria = Iteration criteria flag based on:
%                 1 = deformation under wheels
%                 2 = BM of whole beam (Default)
%             Calc.Proc.Iter.criteria_text = String indicating name of convergence criteria:
%                 'Beam deformation under wheels' for Calc.Proc.Iter.criteria = 1
%                 'Whole beam BM' for Calc.Proc.Iter.criteria = 2
%             Calc.Proc.Iter.tol = Tolerance for iteration stopping criteria (Default: Calc.Constant.tol)
%             Calc.Proc.Iter.continue = Auxiliary variable to indicate if an additional run is necessary (only FI procedure)
%             Calc.Proc.Iter.num = Total number of iterations performed (only FI procedure)
%             Calc.Proc.Iter.BM_diff = Array with the difference in BM between two consecutive iterations (only FI procedure)
%             Calc.Proc.Iter.num_t_bri = Array of indicating how many iterations were performed in each time step (only SSI procedure)
%             Calc.Proc.Iter.max_iter_reached_t_bri = Array indicating if the maximum number of iterations was reached on a given time step (only SSI procedure)
%             Calc.Proc.Iter.mean_num = Average number of iterations per time step for the whole simulation (only SSI procedure)
%             Calc.Proc.Iter.max_reached_num = Logical flag indicating if the maximum number of steps has been reach for any time step (only SSI procedure)
%             Calc.Proc.Iter.max_reached_num_per = Calc.Proc.max_reached_num expressed in percentage (only SSI procedure)

% -------------------------------------------------------------------------
% ------------------------------- Veh -------------------------------------
% -------------------------------------------------------------------------

% Veh = Structure with the variables for all the vehicles
%     NOTE: The information for i-th vehicle is stored in Veh(i)
%     Veh.Model = Substructure with information about the vehicle model
%         Veh.Model.type = Name of function to run to generate the vehicle's system matrices
%         Veh.Model.function_path = Path of the location of function to generate the vehicle model
%     Veh.Prop = Substructure with information about the particular vehicle properties
%         Veh.Prop.mBi = Array of body masses; [mB1, mB2, ...]
%         Veh.Prop.IBi = Array of body momments of intertia; [IB1, IB2, ...]
%         Veh.Prop.kSj = Array of suspension stiffness; [kS1, kS2, ...]
%         Veh.Prop.cSj = Array of suspension viscous damping; [cS1, cS2, ...]
%         Veh.Prop.mGj = Array of axle/group mass; [mG1, mG2, ...]
%         Veh.Prop.IGj = Array of axle/group moment of inertia (if it is an axle group); [IG1, IG2, ...]
%         Veh.Prop.kTk = Array of tyre stiffness; [kT1, kT2, ...]
%         Veh.Prop.cTk = Array of tyre viscous damping; [cT1, cT2, ...]
%         Veh.Prop.ai = Array of distance of body mass centre of gravity to the front of the body; [a1, a2, ...]
%         Veh.Prop.bi = Array of distance of body mass centre of gravity to the back of the body; [b1, b2, ...]
%         Veh.Prop.dj = Array of x xoordinate of axle/group to centre of gravity of its corresponding body; [d1, d2, ...]
%         Veh.Prop.ek = Array of x coordinate of wheel to centre of gravity of its corresponding axle/group; [e1, e2, ...]
%         Veh.Prop.ax_sp = Array of spacing of each tyre to the next; [0 ax_sp2, ax_sp3, ...]
%         Veh.Prop.ax_dist = Array of distance of each tyre to the 1st tyre; [0 ax_dist2, ax_dist3, ...]
%         Veh.Prop.wheelbase = Distance between 1st and last wheel
%         Veh.Prop.num_wheels = Total number of wheels
%     Veh.Pos = Substructure with information about the vehicle position, time and profile
%         Veh.Pos.x0 = Initial position of 1st axle of vehicle (from left bridge support)
%         Veh.Pos.vel = Velocity of vehicle [m/s]
%             Positive value = vehicle moving from left to right
%             Negative value = vehicle moving from right to left
%         Veh.Pos.vel_sign = Sign of the vehicle's velocity
%             Positive value = vehicle moving from left to right
%             Negative value = vehicle moving from right to left
%         Veh.Pos.vel_values = Array with velocity values of the vehicle at various points along the x-axis
%         Veh.Pos.x0_values = Array with x-coordinates where Veh.Pos.vel_values are defined
%         Veh.Pos.t_values = Array with corresponding time values when the first wheel arrives at x0_values
%         Veh.Pos.a_values = Array with corresponding interval cte acceleration
%         Veh.Pos.vel_values_t = Instantaneous velocity of the vehicle. Array of size [1,Calc.Solver.num_t]
%         Veh.Pos.var_vel_flag = Flag indicating if the vehicle is moving with variable velocity (1=yes)
%         Veh.Pos.definition_option = Number indicating how is the speed of the vehicle defined
%             1 = Constant speed: Defined by .x0 and .vel
%             2 = Variable speed: Defined by .vel_values and .x0_values
%             3 = Variable speed: Defined by [.x0, .vel, .a] and constant speed at approach (.vel) and starts acceleration (.a) when reaching the bridge
%         Veh.Pos.prof_x0 = X coordinate of the tyre furthest away from the beam at start of the event
%         Veh.Pos.prof_x_end = X coordinate of the tyre furthest away from the beam at end of the event
%         Veh.Pos.min_t_end = Minimum time requried to simulate the crossing of the vehicle
%         Veh.Pos.wheels_x = Matrix with X coordinate for each wheel in time; size = [num_wheels,num_t]
%         Veh.Pos.wheels_on_beam = Matrix with flags indicating the presence on the beam of each wheel in time; size = [num_wheels,num_t]
%         Veh.Pos.t0_ind_beam = time step number when the firt wheel enters the beam
%         Veh.Pos.t_end_ind_beam = time step number when the last wheel leaves the beam
%         Veh.Pos.wheels_h = Matrix with profile elevations for each wheel in time; size = [num_wheels,num_t]
%         Veh.Pos.wheels_hd = Matrix with profile elevations first derivative in time for each wheel in time; size = [num_wheels,num_t]
%         Veh.Pos.elexj = The beam element number for each wheel in time
%         Veh.Pos.xj = The relative position with respect to start of the beam element elexj for each wheel in time
%     Veh.Event = Substructure with information about the event
%         Veh.Event.max_vel = Maximum vehicle velocity of all the vehicles in the event
%         Veh.Event.num_veh = Number of vehicles involved in the event
%     Veh.DOF = Substructure with information about the DOFs of the vehicle model
%         NOTE: The information about the j-th DOF is stored in Veh.DOF(j)
%         Veh.DOF.dependency = Text specifiying if that DOF is dependent/independent
%         Veh.DOF.name = Text with the name of the DOF
%         Veh.DOF(1).num_dependent = number of dependent DOFs (Note, information stored only in Veh.DOF(1))
%         Veh.DOF(1).num_independent = number of independent DOFs (Note, information stored only in Veh.DOF(1))
%         Veh.DOF.type = Text stating the type of DOF (displacement/rotational)
%     Veh.SysM = Substructure with the vehicle's system matrices
%         Veh.SysM.M = Mass matrix of the vehicle model
%         Veh.SysM.C = Damping matrix of the vehicle model
%         Veh.SysM.K = Stiffness matrix of the vehicle model
%         Veh.SysM.N2w = Matrix that relates the nodal displacements (DOFs) to the displacements of the 
%             top of the wheels. This is relevant for vehicle with axle groups.
%     Veh.Static = Substructure with information about the vehicle's static configuration
%         Veh.Static.F_vector_no_grav = Array of masses to use to calculate the static load of the vehicle; size = [1, num_DOF]
%             Multiply it by gravity to get the force to apply to each DOF.
%         Veh.Static.load = Array of static vertical forces on the road; size = [num_wheels, 1]
%         Veh.Static.check = Internal check result of static force calculation (1=OK)
%         Veh.Static.GVW_N = Gross vehicle weight (GVW) in [N]
%     Veh.Modal = Substructure containing the results from the modal analysis of the vehicle
%         Veh.Modal.w = Array of circular frequencies of vehicle [rad/s]; size = [1, num_DOF]
%         Veh.Modal.f = Array of natural frequencies of vehicle [Hz]; size = [1, num_DOF]

% -------------------------------------------------------------------------
% ------------------------------- Beam ------------------------------------
% -------------------------------------------------------------------------

% Beam = Structure with information about the beam and its modelling options
%     Beam.Prop = Substructure with information about the beam's mechanical properties
%         Beam.Prop.type = Specify the type of beam bridge
%             0 = Custom definition of properties
%             'T' = T type beam bridge
%             'Y' = Y type beam bridge
%             'SY' = Super-Y type beam bridge
%         Beam.Prop.Lb = Total length of beam [m]
%         Beam.Prop.rho = Mass per unit length [kg/m]
%         Beam.Prop.E = Modulus of elasticity [N/m^2]
%         Beam.Prop.I = Second moment of area [m^4]
%         Beam.Prop.A = Cross sectional area [m^2]
%         Beam.Prop.damp_per = Percentage of bridge damping
%         Beam.Prop.damp_xi = Damping ratio (damp_per/100)
%         Beam.Prop.E_n = Array of values giving the E for each element; size = [1,num. of elements]
%         Beam.Prop.I_n = Array of values giving the I for each element; size = [1,num. of elements]
%         Beam.Prop.rho_n = Array of values giving the rho for each element; size = [1,num. of elements]
%         Beam.Prop.A_n = Array of values giving the A for each element; size = [1,num. of elements]
%     Beam.BC = Substructure with information about the beam's boundary conditions
%         Beam.BC.loc = Array listing locations of supports in X direction; [xBC1, xBC2, ...]
%         Beam.BC.vert_stiff = Vertical stiffnes value of each of the supports
%             0 = Free vertical displacement
%             value = Vertical stiffness value of the support
%             Inf = Fixed, no displacement
%         Beam.BC.rot_stiff = Rotational stiffnes value of each of the supports
%             0 = Free rotation
%             value = Rotational stiffness value of the support
%             Inf = Fixed, no rotation
%         Beam.BC.supp_num = Number of supports
%         Beam.BC.loc_ind = Node number closest to the support
%         Beam.BC.DOF_fixed = Array listing DOFs with fixed boundary condition
%         Beam.BC.DOF_with_values = Array listing DOFs that have some additional stiffness
%         Beam.BC.DOF_stiff_values = Array with stiffness values to be added to the DOFs specified in Beam.BC.DOF_with_values
%         Beam.BC.num_DOF_fixed = Number of fixed DOFs
%         Beam.BC.num_DOF_with_values = Number of DOFs with additional stiffness
%         Beam.BC.DOF_fixed_value = Value to use in the diagonal element when the DOF is fixed
%     Beam.Mesh = Substructure with information about the beam mesh
%         Beam.Mesh.Ele = Substructure with information about the elements of the mesh
%             Beam.Mesh.Ele.num = Total number of elements in the model
%             Beam.Mesh.Ele.a = Array with each element X dimension
%             Beam.Mesh.Ele.acum = X coordinate of each node
%             Beam.Mesh.Ele.nodes = Matrix of size [number of elements, num. nodes per element]
%                 Each row includes the indices of the nodes for each element. 
%                 This variable is more useful when using more complex elements (not included here)
%             Beam.Mesh.Ele.DOF = Matrix of size [number of elements, num. DOF per element]
%                 Each row includes the DOF asociated to every element. Each element represents a row.
%             Beam.Mesh.Ele.shape_fun = Anonymous function of the element's shape function.
%                 Inputs [x = local x coordinate, a =element size]
%             Beam.Mesh.Ele.shape_fun_p = Anonymous function of the first derivative (prime) element's shape function. 
%                 Inputs [x = local x coordinate, a =element size]
%             Beam.Mesh.Ele.shape_fun_pp = Anonymous function of the second derivative (double-prime) element's shape function. 
%                 Inputs [x = local x coordinate, a =element size]
%         Beam.Mesh.Node = Substructure with information about the nodes of the mesh
%             Beam.Mesh.Node.at_mid_span = Node at mid-span
%             Beam.Mesh.Node.coord = Coordinates of all nodes [X coord], one row for each node.
%             Beam.Mesh.Node.num_perEle = Nodes per Element
%             Beam.Mesh.Node.num = Total number of nodes in the model
%         Beam.Mesh.DOF = Substructure with information about the DOFs of the mesh
%             Beam.Mesh.DOF.num = Total number of DOF of the mesh
%             Beam.Mesh.DOF.num_perNode = Number of DOF per node
%     Beam.Damage = Substructure with information about the beam damage
%         Beam.Damage.type = Number indicating the type of damage
%             0 = No damage
%             1 = One element damage
%             2 = Global damage
%             3 = Change in boundary conditions
%             4 = Function
%         For Beam.Damage.type = 0 
%             No damage
%         For Beam.Damage.type = 1 
%             One element damage
%             Beam.Damage.loc_per = Location of the damage in percentage of beam length
%             Beam.Damage.E.per = Magnitude of Stiffnes reduction in percentage
%             Beam.Damage.I.per = Magnitude of Inertia reduction in percentage
%             Beam.Damage.rho.per = Magnitude of Mass per unit length reduction in percentage
%         For Beam.Damage.type = 2 
%             Global damage
%             Beam.Damage.E.per = Magnitude of Stiffnes reduction in percentage
%             Beam.Damage.I.per = Magnitude of Inertia reduction in percentage
%             Beam.Damage.rho.per = Magnitude of Mass per unit length reduction in percentage
%         For Beam.Damage.type = 3 
%             Change in boundary conditions
%             Beam.Damage.BC.loc = Array with location of BCs to change (Default Beam.BC.loc)
%             Beam.Damage.BC.vert_stiff = Array with vertical stiffness to add (Optional)
%             Beam.Damage.BC.rot_stiff = Array with rotational stiffness to add (Optional)
%             These stiffness values can be:
%                 negative = to reduce the stiffness of a given configuration
%                 positive = to increase the stiffness of a given configuration
%                 Inf = to fix the correspoinding DOF
%         For Beam.Damage.type = 4 
%             Variation in beam E defined via a custom function
%             Beam.Damage.E.Inputs.values = Inputs to the user defined function
%             Beam.Damage.E.fun = Function that defines the spatial variation of a factor to apply to the original E property
%             Beam.Damage.Plot.on = Switch on/off graphical representation of the factor applied to E (Default: 0 = off)
%     Beam.SysM = Substructure with information about the beam's system matrices
%         Beam.SysM.K = Global stiffness matrix
%         Beam.SysM.C = Global damping matrix
%         Beam.SysM.M = Global mass matrix
%     Beam.Modal = Substructure with information about beam's modal analysis
%         Beam.Modal.num_rigid_modes = Number of rigid modes
%         Beam.Modal.modes = Modes of vibration of in columns; size = [Num. of DOF, Num. of modes]
%         Beam.Modal.w = Circular frequencies of the beam [rad/s] (column vector)
%         Beam.Modal.f = Natural frequencies of the beam [Hz] (column vector)

% -------------------------------------------------------------------------
% ------------------------------- Sol -------------------------------------
% -------------------------------------------------------------------------

% Sol = Structure containing the results from the simulation
%     Sol.Veh = Substructure containing results for the vehicle(s)
%         NOTE: The information for i-th vehicle is stored in Sol.Veh(i)
%         Sol.Veh.U = Vehicle's DOF displacements (or rotations)
%             size = [Num of vehicle DOFs, Num. of simulation time steps]
%         Sol.Veh.V = Vehicle's DOF velocities
%             size = [Num of vehicle DOFs, Num. of simulation time steps]
%         Sol.Veh.A = Vehicle's DOF accelerations
%             size = [Num of vehicle DOFs, Num. of simulation time steps]
%         Sol.Veh.Under = Substructure containing results for the vehicle(s) wheel contact point(s)
%             At the point where the tyre meets the road
%             Sol.Veh.Under.def = Beam deformation under the wheels
%                 size = [Number of wheels, Num. of simulation time steps on the beam]
%             Sol.Veh.Under.vel = Beam's velocity under the wheels
%                 size = [Number of wheels, Num. of simulation time steps on the beam]
%             Sol.Veh.Under.h = Profile elevation under the wheels
%                 size = [Number of wheels, Num. of simulation time steps on the beam]
%             Sol.Veh.Under.hd = 1st time derivative of profile elevation under the wheels
%                 size = [Number of wheels, Num. of simulation time steps on the beam]
%             Sol.Veh.Under.onBeamF = Force of each wheel acting on the beam
%                 size = [Number of wheels, Num. of simulation time steps on the beam]
%         Sol.Veh.Wheels = Substructure containing results for the vehicle(s) wheel displacement and velocities
%             These are the vehicle responses at the top of the tyre
%             For wheels not in an axle group, this is the same as the suspension displacement and velocities
%             Sol.Veh.Wheels.U = Top of the tyre displacements
%                 size = [Num of wheels, Num. of simulation time steps]
%             Sol.Veh.Wheels.Urel = Relative displacements between top and bottom of tyre (Wheels' relative displacements)
%                 size = [Num of wheels, Num. of simulation time steps]
%             Sol.Veh.Wheels.V = Top of the tyre velocities
%                 size = [Num of wheels, Num. of simulation time steps]
%             Sol.Veh.Wheels.Vrel = Relative velocities between top and bottom of tyre (Wheels' relative velocities)
%                 size = [Num of wheels, Num. of simulation time steps]
%     Sol.Beam = Substructure with results for the beam
%         Sol.Beam.(LoadEffect) = Substructure with "LoadEffect" results for the beam
%             Several load effects are considered, with following field names:
%                 U = Deformation of each DOF
%                 V = Velocity of each DOF
%                 A = Acceleration of each DOF
%                 BM = Bending moment of each node
%                 Shear = Shear forces of each node
%                 U_static = Static deformation of each node
%                 BM_static = Static bending moment of each node
%                 Shear_static = Static shear forces of each node
%                 Disp = Vertical displacement of each node
%                 Disp_static = Static vertical displacement of each node
%                 Disp_dot = 1st time derivative of vertical displacement of each node
%                 Disp_ddot = 2nd time derivative of vertical displacement of each node
%                 Rot = Rotation of each node
%                 Rot_static = Static rotation of each node
%                 Rot_dot = 1st time derivative of rotation of each node
%                 Rot_ddot = 2nd time derivative of rotation of each node
%             Sol.Beam.(LoadEffect).value_DOFt = Result for each DOF in time; size = [num of DOF, num. of simulation time step on beam]
%             Sol.Beam.(LoadEffect).value_xt = Vertical displacement for each node in time; size = [num of DOF, num. of simulation time step on beam]
%             Sol.Beam.(LoadEffect).Max.value = Maximum value
%             Sol.Beam.(LoadEffect).Max.COP = Location of maximum along the beam (Critical Observation Point)
%             Sol.Beam.(LoadEffect).Max.pCOP = COP in percentage of beam length
%             Sol.Beam.(LoadEffect).Max.cri_t_in = Time step when the maximum occurred
%             Sol.Beam.(LoadEffect).Max.value05 = Maximum value at mid-span (if mid-span node exists)
%             Sol.Beam.(LoadEffect).Min.value = Minimum value
%             Sol.Beam.(LoadEffect).Min.COP = Location of minimum along the beam (Critical observation point)
%             Sol.Beam.(LoadEffect).Min.pCOP = COP in percentage of beam length
%             Sol.Beam.(LoadEffect).Min.cri_t_in = Time step when the minimum occurred
%             Sol.Beam.(LoadEffect).Min.value05 = Minimum value at mid-span (if mid-span node exists)

% ---- End of script ----