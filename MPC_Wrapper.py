# coding: utf8

import numpy as np
import MPC
import FootstepPlanner
from multiprocessing import Process, Value, Array


class MPC_Wrapper:
    """Wrapper to run FootstepPlanner + MPC on another process

    Args:
        dt (float): Time step of the MPC
        n_steps (int): Number of time steps in one gait cycle
        multiprocessing (bool): Enable/Disable running the MPC with another process
    """

    def __init__(self, dt, n_steps, multiprocessing=False):

        self.f_applied = np.zeros((12,))

        self.multiprocessing = multiprocessing
        if multiprocessing:
            self.newData = Value('b', False)
            self.newResult = Value('b', False)
            self.dataIn = Array('d', [0.0] * 329)
            self.dataOut = Array('d', [0] * 12)
            self.fsteps_future = np.zeros((6, 13))
        else:
            # Create the new version of the MPC solver object
            self.mpc = MPC.MPC(dt, n_steps)

    def run_MPC(self, dt, n_steps, k, T_gait, t_stance, joystick, fstep_planner, mpc_interface):

        if self.multiprocessing:
            self.run_MPC_asynchronous(dt, n_steps, k, T_gait, t_stance, joystick, fstep_planner, mpc_interface)
        else:
            self.run_MPC_synchronous(dt, n_steps, k, T_gait, t_stance, joystick, fstep_planner, mpc_interface)

    def get_latest_result(self, k):

        if (k != 0):
            if self.multiprocessing:
                if self.newResult.value:
                    self.newResult.value = False
                    return self.convert_dataOut()
                else:
                    raise ValueError("Error: something went wrong with the MPC, result not available.")
            else:
                return self.mpc.f_applied
        else:
            return np.array([0.0, 0.0, 8.0] * 4)

    def run_MPC_synchronous(self, dt, n_steps, k, T_gait, t_stance, joystick, fstep_planner, mpc_interface):

        # Run the MPC to get the reference forces and the next predicted state
        # Result is stored in mpc.f_applied, mpc.q_next, mpc.v_next
        self.mpc.run((k/20), T_gait, t_stance,
                     mpc_interface.lC, mpc_interface.abg, mpc_interface.lV, mpc_interface.lW,
                     mpc_interface.l_feet, fstep_planner.xref, fstep_planner.x0, joystick.v_ref,
                     fstep_planner.fsteps)

        # Output of the MPC
        self.f_applied = self.mpc.f_applied

    def run_MPC_asynchronous(self, dt, n_steps, k, T_gait, t_stance, joystick, fstep_planner, mpc_interface):

        if (k == 0):
            p = Process(target=self.create_MPC_asynchronous, args=(self.newData, self.newResult, self.dataIn, self.dataOut))
            p.start()

        # print("Setting Data")

        self.compress_dataIn(dt, n_steps, k, T_gait, t_stance, joystick, fstep_planner, mpc_interface)

        print("Sending")
        """print(dt, n_steps, k, T_gait, t_stance)
        print(mpc_interface.lC.ravel())
        print(mpc_interface.abg.ravel())
        print(mpc_interface.lV.ravel())
        print(mpc_interface.lW.ravel())
        print(mpc_interface.l_feet.ravel())
        print(joystick.v_ref.ravel())
        print(fstep_planner.fsteps)"""
        #print(fstep_planner.xref)
        self.newData.value = True

        return 0

    def create_MPC_asynchronous(self, newData, newResult, dataIn, dataOut):

        # dt, n_steps = self.decompress_dataIn()

        # Create the new version of the MPC solver object
        # mpc = MPC.MPC(dt, n_steps)
        # print("Entering infinite loop")
        while True:
            if newData.value:
                newData.value = False
                # print("New data detected")

                print("Receiving")
                dt, nsteps, k, T_gait, t_stance, lC, abg, lV, lW, l_feet, xref, x0, v_ref, fsteps  = self.decompress_dataIn(dataIn)
                dt = dt[0]
                nsteps = np.int(nsteps[0])
                k = k[0]
                T_gait = T_gait[0]
                t_stance = t_stance[0]
                lC = np.reshape(lC, (3, 1))
                abg = np.reshape(abg, (3, 1))
                lV = np.reshape(lV, (3, 1))
                lW = np.reshape(lW, (3, 1))
                l_feet = np.reshape(l_feet, (3, 4))
                xref = np.reshape(xref, (12, nsteps+1))
                x0 = np.reshape(x0, (12, 1))
                v_ref = np.reshape(v_ref, (6, 1))
                fsteps = np.reshape(fsteps, (6, 13))

                """print(dt, nsteps, k, T_gait, t_stance)
                print(lC.ravel())
                print(abg.ravel())
                print(lV.ravel())
                print(lW.ravel())
                print(l_feet.ravel())
                print(v_ref.ravel())
                print(fsteps)"""
                #print(xref)

                #print("Roll")
                #print(fsteps)
                #print("into")
                #self.roll_asynchronous(fsteps)
                #print(self.fsteps_future)
                #print(xref[0:6,0:3])

                if k == 0:
                    loop_mpc = MPC.MPC(dt, nsteps)

                loop_mpc.run((k/20), T_gait, t_stance, lC, abg, lV, lW,
                             l_feet, xref, x0, v_ref, fsteps)

                self.dataOut[:] = loop_mpc.f_applied.tolist()

                newResult.value = True

        return 0

    def compress_dataIn(self, dt, n_steps, k, T_gait, t_stance, joystick, fstep_planner, mpc_interface):

        # print("Compressing dataIn")

        fstep_planner.fsteps[np.isnan(fstep_planner.fsteps)] = 0.0

        self.dataIn[:] = np.concatenate([[dt, n_steps, k, T_gait, t_stance], np.array(mpc_interface.lC).ravel(), np.array(mpc_interface.abg).ravel(),
                         np.array(mpc_interface.lV).ravel(), np.array(mpc_interface.lW).ravel(), np.array(mpc_interface.l_feet).ravel(), fstep_planner.xref.ravel(), fstep_planner.x0.ravel(), joystick.v_ref.ravel(),
                         fstep_planner.fsteps.ravel()], axis=0)

        return 0.0
        # guess = 5 + 3 + 3 + 3 + 3 + 12 + (n_steps+1) * 12 + 12 + 6 + 13 * 6
        # np.concatenate([A, A], )
        #test[np.isnan(test)] = 0.0
        #return test.tolist()

    def decompress_dataIn(self, dataIn):

        # print("Decompressing dataIn")

        sizes = [0, 1, 1, 1, 1, 1, 3, 3, 3, 3, 12, (np.int(dataIn[1])+1) * 12, 12, 6, 13*6]
        csizes = np.cumsum(sizes)
        return [dataIn[csizes[i]:csizes[i+1]] for i in range(len(sizes)-1)]

        # return dataIn[0], dataIn[1], dataIn[2], dataIn[3]

    def convert_dataOut(self):

        return np.array(self.dataOut[:])

    def roll_asynchronous(self, fsteps):
        """Move one step further in the gait cycle. Since the output of the asynchronous MPC is retrieved by
        TSID during the next call to the MPC, it should not work with the current state of the gait but with the
        gait on step into the future. That way, when TSID retrieves the result, it is consistent with the current
        state of the gait.

        Decrease by 1 the number of remaining step for the current phase of the gait and increase
        by 1 the number of remaining step for the last phase of the gait (periodic motion).
        Simplification: instead of creating a new phase if required (see roll function of FootstepPlanner) we always
        increase the last one by 1 step. That way we don't need to call other functions to predict the position of
        footstep when a new phase is created.
        """

        self.fsteps_future = fsteps.copy()

        # Index of the first empty line
        index = next((idx for idx, val in np.ndenumerate(self.fsteps_future[:, 0]) if val==0.0), 0.0)[0]

        # Create a new phase if needed or increase the last one by 1 step
        self.fsteps_future[index-1, 0] += 1.0

        # Decrease the current phase by 1 step and delete it if it has ended
        if self.fsteps_future[0, 0] > 1.0:
            self.fsteps_future[0, 0] -= 1.0
        else:
            self.fsteps_future = np.roll(self.fsteps_future, -1, axis=0)
            self.fsteps_future[-1, :] = np.zeros((13, ))

        return 0
