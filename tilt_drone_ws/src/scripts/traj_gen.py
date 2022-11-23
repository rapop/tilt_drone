
import numpy as np
from math import isnan
from scipy.linalg import block_diag, lu_factor, lu_solve, lu
#from qpsolvers import solve_qp
#import quadprog
#import scipy.optimize as opt
#import cvxpy as cp

class TrajectoryGenerator(object):
    def __init__(self, wps, t, dt, constraints):
        '''
        implementation of the trajectory generation part of the paper Minimum Snap Trajectory Generation
        and Control for Quadrotors by Daniel Mellinger and Vijay Kumar . With inspiration from
        Adam P. Bry's PhD thesis and a paper by Richter and al. on Polynomial Trajectory Planning

        Arguments:
        wps -- Number of waypoints (not including initial conditions)
        t -- time vector, starts with 0
        constraints -- array of position, velocity, acc, jerk and snap constraints for every state

        Returns:
        trajectory -- a array of t, x_des, dx_des, ddx_des, dddx_des, ddddx_des,
        y_des, dy_des, ddy_des, dddy_des, ddddy_des, z_des, dz_des, ddz_des, dddz_des,
        ddddz_des, psi_des, dpsi_des,ddpsi_des, dddpsi_des, ddddpsi_des
        for each discretization point.
        '''
        n = 6 #Order of the polynomials of a trajectory
        r = 4 # Order of derivative of the states
        states = 4 # x,y,z,psi states

        # find coeffs
        coeffs = self._solve_kkt(n,wps,r,constraints,t,states)

        # compute array of trajectory
        self.traj_and_time = self._discretize_trajectory(coeffs,n,wps,r,states,dt,t)

    def return_array(self):
        return self.traj_and_time

    def _discretize_trajectory(self,coeffs,n,wps,r,states,dt,t):
        alpha = 1
        dt = dt*alpha
        t = t*alpha

        n_coeffs = n+1
        polynomial = np.zeros((n_coeffs, wps, r+1, states))
        for state in range(1,states+1):
            polynomial[:,:, 0, state-1] = np.transpose(np.reshape(np.expand_dims(coeffs[:,state-1],axis=1), (wps, n_coeffs)))
            scale_vec = [(1/alpha)**x for x in range(n, -1, -1)]
            scale_vec = np.transpose(np.expand_dims(np.asarray(scale_vec),axis=0))
            b = np.ones((n_coeffs,wps-1))
            scale_vec = np.append(scale_vec, b, axis=1)
            polynomial[:,:, 0, state-1] = scale_vec * polynomial[:,:, 0, state-1]

            for der in range(1,r+1):
                for wp in range(1,wps+1):
                    poly_temp = polynomial[:, wp-1, der-1, state-1]
                    poly_temp = np.expand_dims(np.array(np.polyder(np.poly1d(np.squeeze(poly_temp)))),axis = 0)
                    a = np.zeros((1, n_coeffs-np.shape(poly_temp)[1]))
                    poly_temp = np.append(a , poly_temp, axis=1)
                    poly_temp = np.transpose(poly_temp)
                    polynomial[:, wp-1, der, state-1] = np.squeeze(poly_temp)

        time = np.arange(0,t[-1]+dt,dt)
        traj = np.zeros((len(time), r+1, states))
        for state in range(1,states+1):
            for t2 in range(1,len(time)+1):
                if time[t2-1] == 0:
                    wp = 1
                else:
                    wp = np.where(time[t2-1]>t)[0][-1] + 1
                for der in range(1,r+2):
                    scaledt = (time[t2-1] - t[wp-1]) / (t[wp] - t[wp-1])
                    scaledt = scaledt * alpha
                    poly_temp = polynomial[:, wp-1, der-1, state-1]
                    traj[t2-1, der-1, state-1] = np.polyval(poly_temp, scaledt)

        traj_and_time = np.append(np.expand_dims(np.transpose(time),axis=1) , traj[:,:,0], axis=1)
        for i in range(1,states):
            traj_and_time = np.append(traj_and_time,traj[:,:,i], axis=1)

        return traj_and_time


    def _solve_kkt(self,n,wps,r,constraints,t,states):
        c_shape = ((n+1)*wps,states)
        coeffs = np.empty(c_shape)
        # build Hessien
        H = self._buildh(n, wps, r, t)

        for i in range(1,states+1):

            # build constraints
            Aeq, beq = self._buildConstraints(n, wps, r, constraints[:,:,i-1], t)
            # remove last 2 constraints for psi state
            Aeq = Aeq[0:13,:]
            beq = beq[0:13,:]
            '''
            * build the KKT conditions
            * We want to solve
            *
            *  [ H A'  [x*       = [0
            *    A 0 ]  lambda*]    b]
            '''
            K =  np.block([[H,  np.transpose(Aeq)],
                        [Aeq, np.zeros((np.shape(Aeq)[0], np.shape(Aeq)[0])) ]])

            B = np.block([[np.zeros((np.shape(H)[0], np.shape(beq)[1]))],
                                [beq]])

            #LU decomposition
            LU, P = lu_factor(K)
            x = lu_solve((LU,P),B)
            # x = solve_qp(H, q,A=Aeq, b=beq)
            # q = np.zeros((14,))
            # # x = solvers.qp(matrix(H), matrix(q), None, None, A =matrix(Aeq), b = matrix(beq))
            # print(np.shape(np.squeeze(beq)))
            # x = quadprog.solve_qp(H, q, Aeq, np.squeeze(beq))
            # P, L, U = lu(K)
            # # y = np.matmul(np.linalg.inv(L),np.matmul(P,B))
            # # x = np.matmul(np.linalg.inv(U),y)
            # y = np.linalg.solve(L, np.matmul(P,B))
            # x = np.linalg.solve(U, y)
            # print(np.shape(x))

            # # defining optimization vairables
            # V = Variable(len(H))
            # # setting up the optimization problem
            # objective = Minimize(quad_form(V,H)) #see https://github.com/cvxr/CVX/blob/master/functions/quad_form.m
            # print(np.shape(Aeq))
            # print(np.shape(beq))
            # print(np.shape(V))
            # constraints = [Aeq*V == np.squeeze(beq)] # constraints defined by waypoints
            # prob = Problem(objective, constraints)
            # #solving the optimization problem
            # prob.solve(solver='SCS', eps=1e-12)
            # # return results
            # x = np.array(V.value)
            # print(x)


            # x = cp.Variable(len(H))
            # q0 = np.ones(len(H))
            # print(np.shape(H))
            # # H = np.dot(H.T,H)
            # print(np.shape(Aeq))
            # print(np.shape(beq))
            # Aeq = Aeq[0:13,:]
            # beq = beq[0:13,0]
            # prob = cp.Problem(cp.Minimize((1/2)*cp.quad_form(x, H)),[Aeq*x == np.squeeze(beq)])
            # prob.solve(solver = 'CVXOPT', verbose = True)
            # y = x.value
            # print(prob.value)
            # print(prob.status)
            # print(y)

            # # Generate a random non-trivial quadratic program.
            # m = 15
            # n = 10
            # p = 5
            # np.random.seed(1)
            # P = np.random.randn(n, n)
            # P = np.dot(P.T,P)
            # q = np.random.randn(n)
            # G = np.random.randn(m, n)
            # h = G*np.random.randn(n)
            # A = np.random.randn(p, n)
            # b = np.random.randn(p)
            #
            # Define and solve the CVXPY problem.
            # x = cp.Variable(n)
            # prob = cp.Problem(cp.Minimize((1/2)*cp.quad_form(x, P)),
            #                  [A*x == b])
            # prob.solve()
            #
            # # Print result.
            # print("\nThe optimal value is", prob.value)
            # print("A solution x is")
            # print(x.value)
            # print("A dual solution corresponding to the inequality constraints is")
            # print(prob.constraints[0].dual_value)
            #
            # sol = solve_qp(H, [], None, None, Aeq, beq)

            # construct maxtrix coeffs
            coeffs[:,i-1] = np.squeeze(x[0:(n+1)*wps])
            # coeffs[:,i-1] = y

        return coeffs

    def _buildh(self, n, wps, r, t):
        num_coeffs = n + 1
        H = []
        for wp in range(1,wps+1):
            H_r = np.zeros((num_coeffs,num_coeffs))
            t0 =  t[wp-1]
            tend = t[wp]
            for i in range(n+1):
                for l in range(n+1):
                    if i >= r and l >= r :
                        cum_mul = 1
                        for m in range(r):
                            cum_mul = cum_mul * (i-m) * (l-m)
                        H_r[i,l] = cum_mul
                        H_r[i,l] = H_r[i,l] / (i+l-2*r+1)
                    else:
                        H_r[i,l] = 0
            H_r =  1/((t[wp]-t[wp-1])**(2*r-1)) * H_r
            H_r = np.rot90(np.rot90(H_r))
            if wp == 1 :
                H = H_r
            else :
                H = block_diag(H, H_r)
        return H

    def _buildConstraints(self, n, wps, r, constraints, t):
        n_coeffs = n + 1
        n_wps = wps + 1
        coeffs = self._getCoefficientMatrix(n, r)
        I = np.rot90(np.eye(n_coeffs))
        A_0 = []
        b_0 = []
        for wp in range(1,n_wps+1):
            for der in range(1,r+1):
                if isnan(constraints[der-1,wp-1]) == False:
                    if wp == 1:
                        a = np.zeros((n_wps-1) * n_coeffs)
                        int_t = 1 / (t[wp] - t[wp-1])**(der-1)
                        polynomial = coeffs[der-1, :] * I[der-1, :] * int_t
                        idx = (wp-1) * n_coeffs + 1
                        a[idx-1:idx+n] = polynomial
                        b = constraints[der-1, wp-1]
                        A_0.append(a)
                        b_0.append(b)
                    elif wp == n_wps:
                        a = np.zeros((n_wps-1) * n_coeffs)
                        int_t_next = 1 / (t[wp-1] - t[wp-2])**(der-1)
                        polynomial = coeffs[der-1, :] * int_t_next
                        idx = (wp-2) * n_coeffs + 1
                        a[idx-1:idx+n] = polynomial
                        b = constraints[der-1, wp-1]
                        A_0.append(a)
                        b_0.append(b)
                    else:
                        a = np.zeros((2, (n_wps-1) * n_coeffs))
                        int_t_next = 1 / (t[wp-1] - t[wp-2])**(der-1)
                        poly = coeffs[der-1, :] * int_t_next
                        idx = (wp-2) * n_coeffs + 1
                        a[0, idx-1:idx+n] = poly
                        int_t = 1 / (t[wp] - t[wp-1])**(der-1)
                        poly = coeffs[der-1, :] * I[der-1, :] * int_t
                        idx = (wp-1) * n_coeffs + 1
                        a[1, idx-1:idx+n] = poly
                        b = np.ones((2, 1))
                        b = b * constraints[der-1, wp-1]
                        A_0.append(a[0,:])
                        A_0.append(a[1,:])
                        b_0.append(b[0][0])
                        b_0.append(b[1][0])

        A_0 = np.array(A_0)
        b_0 = np.array(b_0)

        A_t = []
        b_t = []
        for wp in range(2,n_wps):
            for der in range(1,r+2):
                if isnan(constraints[der-1,wp-1]):
                    a = np.zeros((n_wps-1) * n_coeffs)
                    int_t = 1 / (t[wp-1] - t[wp-2])**(der-1)
                    int_t_next = 1 / (t[wp] - t[wp-1])**(der-1)
                    a_prev = coeffs[der-1, :] * int_t
                    idx_prev = (wp-2) * n_coeffs + 1
                    a[idx_prev-1:idx_prev+n] = a_prev
                    a_next = - coeffs[der-1, :] * I[der-1,:] * int_t_next
                    idx_next = (wp-1) * n_coeffs + 1
                    a[idx_next-1:idx_next+n] = a_next
                    b = 0

                    A_t.append(a)
                    b_t.append(b)

        A_t = np.array(A_t)
        b_t = np.array(b_t)

        b_0 = b_0.reshape(-1,1)
        b_t = b_t .reshape(-1,1)

        Aeq = np.vstack((A_0, A_t))
        beq = np.vstack((b_0, b_t))

        return Aeq, beq

    def _getCoefficientMatrix(self, n, r):
        num_coeffs = n + 1
        num_poly = r + 1
        coefficients = np.zeros([num_poly, num_coeffs])
        polynomial = np.ones([1, num_coeffs])
        for i in range(num_poly):
            coefficients[i,0:np.shape(polynomial)[1]] = polynomial
            polynomial = np.expand_dims(np.array(np.polyder(np.poly1d(np.squeeze(polynomial)))),axis = 0)
        return coefficients

if __name__ == '__main__':
    wps = 2
    t = [0,1.12,2]
    dt = 0.01

    constraints = np.empty([5,wps+1,4])
    # x  constraints
    constraints[:,:,0] = np.array([[1.2, 1.6, 1.2],
                            [0, 1, 0], # velocity constraints
                            [0, 0, 0], # acceleration constraints
                            [0, None, 0], # jerk constraints
                            [0, None, 0]]) # snap constraints
    # y constraints
    constraints[:,:,1] = np.array([[0.65, 0.65, 0.65],
                            [0, 0, 0],
                            [0, 0, 0],
                            [0, None, 0],
                            [0, None, 0]])
    # z constraints
    constraints[:,:,2] = np.array([[1.2, 1.28, 1.2],
                            [0, 0.1, 0],
                            [0, 0, 0],
                            [0, None, 0],
                            [0, None, 0]])
    # yaw constraints
    constraints[:,:,3] = np.array([[0.7854, 0.7854, 0.7854],
                            [0, 0, 0],
                            [0, 0, 0],
                            [0, None, 0],
                            [0, None, 0]])

    Traj = TrajectoryGenerator(wps, t, dt, constraints)
    traj_np_array = Traj.return_array()
