from transformations import *
from transformations import _EPS
import cv2
import sys

######################################################################
# helper function to get a numerical Jacobian

def num_jac(f, x, h=1e-4):

    Jn = None
    n = len(x)

    for i in range(n):
        delta = numpy.zeros_like(x)
        delta[i] = h
        fp = f(x+delta)
        fn = f(x-delta)
        if Jn is None:
            m = len(fp)
            Jn = numpy.zeros((m, n), dtype=numpy.float64)
        Jn[:,i] = (fp-fn)/(2*h)

    return Jn

######################################################################

def printit(n, v):
    if isinstance(v, numpy.ndarray):
        if len(v.shape) > 1:
            print n + ':\n', v
            print
        else:
            print n + ':', v.flatten()
    else:
        print n + ':', v

######################################################################

def title(name):

    print '*'*70
    print 'testing', name
    print '*'*70
    print

######################################################################
# helper function to print things and check equal

def check(na, va, nb, vb):
    printit(na, va)
    printit(nb, vb)
    if not numpy.allclose(va, vb):
        raise RuntimeError(na + ' != ' + nb)
    print na, '==', nb, '?', True
    print

######################################################################
# helper function to test a Jacobian

def check_jacobian(name, f, x, J, h=1e-4):
    
    m,n = J.shape
    assert(len(x) == n)

    Jn = num_jac(f, x, h)
        
    check(name, J, name+'_numeric', Jn)

######################################################################
# helper function to compute 3x3 skew-symmetric cross product matrix

def cross_matrix_3x3(v):
    return numpy.array([
        [   0.0, -v[2],  v[1] ],
        [  v[2],   0.0, -v[0] ],
        [ -v[1],  v[0],   0.0 ]], dtype=v.dtype)

######################################################################
# get matrix form of quaternion product
#
# quaternion_multiply(p, q) = numpy.dot(quaternion_product_matrix(p), q)
# quaternion_multiply(p, q) = numpy.dot(quaternion_product_matrix(q, True), p)
    
def quaternion_product_matrix(q, conj=False): # (110)
    q0 = q[3]
    q123 = q[:3]
    rval = numpy.zeros((4,4))
    if conj:
        rval[:3,:3] = q0*numpy.eye(3) - cross_matrix_3x3(q123)
    else:
        rval[:3,:3] = q0*numpy.eye(3) + cross_matrix_3x3(q123)
    rval[3,3] = q0
    rval[3,:3] = -q123.reshape(3)
    rval[:3,3] = q123.reshape(3)
    return rval

######################################################################
# create a rotation matrix from a rotation vector (angle * unit axis)
    
def rotation_matrix_from_rvec(r):
    angle = numpy.linalg.norm(r)
    if angle > _EPS:
        return rotation_matrix(angle, r/angle)[:3,:3]
    else:
        return numpy.eye(3,dtype=r.dtype)

######################################################################
# create a unit quaternion from a rotation vector (angle * unit axis)

def quaternion_from_rvec(r):
    return quaternion_about_axis(numpy.linalg.norm(r), r)

######################################################################
# if q = quaternion_from_rvec(v)
# then return the 4x3 Jacobian matrix G which is dq/dv

def quaternion_from_rvec_deriv(v): # Equation (212) in Diebel
    nu = numpy.linalg.norm(v)
    cnu2 = numpy.cos(nu/2)
    snu2 = numpy.sin(nu/2)
    a = cnu2*nu - 2*snu2
    if nu > _EPS:
        G1 = (snu2 / (2*nu)) * numpy.vstack( (2*numpy.eye(3),  -v.reshape((1,3)) ) )
        G2 = (a / (2*nu**3)) * numpy.vstack( ( numpy.dot( v.reshape((3,1)),
                                                          v.reshape((1,3)) ),
                                               numpy.zeros((1,3)) ) )
        return G1 + G2
    else:
        return 0.25 * numpy.vstack( ( 2*numpy.eye(3),
                                      -v.reshape((1,3)) ) )

######################################################################
# create a rotation vector (angle * unit axis) from a unit quaternion
    
def rvec_from_quaternion(q):
    theta = numpy.arccos(q[3])
    if theta > _EPS:
        return 2*theta*q[:3]/numpy.sin(theta)
    else:
        return 2*q[:3]

######################################################################
# if v = rvec_from_quaternion(q),
# then return the 3x4 Jacobian matrix H that is dv/dq

def rvec_from_quaternion_deriv(q): # (227)
    q0 = q[3]
    q123 = q[:3]
    denom = 1 - q0**2
    if denom < _EPS:
        return numpy.hstack( ( 2*numpy.eye(3), q123.reshape((3,1)) ) )
    else:
        c = 1.0/denom
        d = numpy.arccos(q0) / numpy.sqrt(denom)
        H = numpy.hstack( (2*d*numpy.eye(3),
                           2*c*(d*q0 - 1)*q123.reshape((3,1)) ) )
        return H

######################################################################
# multiply two rotation vectors by converting to quaternions and back

def rvec_multiply(v, u):

    qv = quaternion_from_rvec(v)
    qu = quaternion_from_rvec(u)

    qr = quaternion_multiply(qv, qu)
    
    return rvec_from_quaternion(qr)

######################################################################
# if r = rvec_multiply(v, u)
# then return the 3x3 Jacobian matrix that is drdv
# note: not well optimized

def rvec_multiply_deriv(v, u):
    qv = quaternion_from_rvec(v)
    qu = quaternion_from_rvec(u)
    qvu = quaternion_multiply(qv, qu)
    H = rvec_from_quaternion_deriv(qvu)
    Qbar = quaternion_product_matrix(qu, True)
    G = quaternion_from_rvec_deriv(v)
    return numpy.dot(H, numpy.dot(Qbar, G))

######################################################################
# compose rotation vector with quaternion and convert back to rotation
# vector.

def rqr(v, q):

    return rvec_from_quaternion(
        quaternion_multiply(quaternion_from_rvec(v), q))

######################################################################
# compute de/dv of
# 
#   e = rvec_from_quaternion(quaternion_multiply(quaternion_from_rvec(v), q))
#
# evaluated at v=0

def rqr_deriv(q):
 
    q0 = q[3]

    denom = 1 - q0**2

    if denom < _EPS:

        return numpy.eye(3)
        
    else:
    
        # this is the somewhat-optimized contcatenation of the correct things
        # from the derivative functions above
        c = 1.0/denom
        d = numpy.arccos(q0) / numpy.sqrt(denom)

        q123 = q[:3]

        k = -c*(d*q0 - 1)
        outer = numpy.dot( q123.reshape((3,1)), q123.reshape((1,3)) )

        inner = q0*numpy.eye(3) - cross_matrix_3x3(q123)

        return k*outer + d*inner

######################################################################
# identity rigid transform (q0, t0)

def xform_identity():
    return ( numpy.array([0., 0., 0., 1.]), numpy.array([0., 0., 0.]) )


######################################################################
# inverse of a rigid transform

def xform_inverse(xform):
    q, t = xform
    qinv = quaternion_inverse(q)
    tinv = -numpy.dot(quaternion_matrix(qinv)[:3,:3], t)
    return (qinv, tinv)

######################################################################
# matrix form of rigid transform

def xform_matrix(xform):
    q, t = xform
    M = quaternion_matrix(q)
    M[:3, 3] = t
    return M

######################################################################
# compose two rigid transformations (normalizing the resulting
# quaternion for numerical stability)

def xform_compose(xform_a, xform_b):
    qa, ta = xform_a
    qb, tb = xform_b
    qab = quaternion_multiply(qa, qb)
    qab /= numpy.linalg.norm(qab)
    tab = ta + numpy.dot(quaternion_matrix(qa)[:3,:3], tb)
    return (qab, tab)

######################################################################
# prefer using xform_matrix if performing several of these

def xform_transform_forward(xform, p):
    q, t = xform
    return t + numpy.dot(quaternion_matrix(q)[:3,:3], p)

######################################################################
# prefer using xform_matrix if performing several of these

def xform_transform_inverse(xform, p):
    q, t = xform
    qinv = quaternion_inverse(q)
    return numpy.dot(quaternion_matrix(qinv)[:3,:3], p - t)

######################################################################
# compose the transformation with a 6x1 twist = (rvec, tvec)

def xform_compose_twist(xform, twist):
    x2 = ( quaternion_from_rvec(twist[0:3]), twist[3:6] )
    return xform_compose(xform, x2)
    
######################################################################
# we want xform_compose_twist(xa, xform_error(xa, xb)) == xb
#
# optionally returns jacobians as well

def xform_error(xform_a, xform_b, return_jacobians=False):

    q, t = xform_compose( xform_inverse(xform_a), xform_b )

    twist = numpy.zeros(6)
    twist[0:3] = rvec_from_quaternion(q)
    twist[3:6] = t

    if not return_jacobians:
        return twist
    else:

        R = quaternion_matrix(q)[:3, :3]

        A = numpy.zeros((6,6))

        A[:3, :3] = -rqr_deriv(q)
        A[3:, 3:] = -numpy.eye(3)
        A[3:, :3] = cross_matrix_3x3(t)
        
        B = numpy.zeros((6,6))
        B[:3,:3] = -A[:3,:3].T
        B[3:,3:] = R

        return twist, A, B


######################################################################

def _test_basics():

    title('basics')

    # create two rotation vectors: unit axis * angle
    v = numpy.array((0.8, 0.1, 0.2))
    u = numpy.array((0.3, 0.4, 0.5))

    # create quaternions from vectors
    q = quaternion_from_rvec(v)
    p = quaternion_from_rvec(u)

    # get rotation vectors back out
    v2 = rvec_from_quaternion(q)
    u2 = rvec_from_quaternion(p)

    check('v', v, 'v2', v2)
    check('u', u, 'u2', u2)

    # make sure we can take derivatives
    Gv = quaternion_from_rvec_deriv(v)  # G is 4x3
    Hq = rvec_from_quaternion_deriv(q)  # H is 3x4

    check_jacobian('Gv', quaternion_from_rvec, v, Gv)
    check_jacobian('Hq', rvec_from_quaternion, q, Hq)

    # make sure we can represent quaternion products as matrix products
    P = quaternion_product_matrix(p, False)
    Q = quaternion_product_matrix(q, True)

    pq = quaternion_multiply(p, q)
    pq2 = numpy.dot(Q, p)
    pq3 = numpy.dot(P, q)

    check('pq', pq, 'pq2', pq2)
    check('pq', pq, 'pq3', pq3)

    # make sure we can multiply rotation vectors together (this
    # converts to quaternions under the hood)
    r = rvec_multiply(u, v)
    r2 = rvec_from_quaternion(pq)

    check('r', r, 'r2', r2)
    
    # make sure we can take the derivative of a rotation vector product
    drdu = rvec_multiply_deriv(u, v) # drdu is 3x3

    check_jacobian('drdu', lambda u: rvec_multiply(u, v), u, drdu)

    # rqr_deriv(q) is the derivative with respect to delta of rvec_from_quaternion(quaternion_multiply(quaternion_from_rvec(delta), q)) evaluated at delta=0
    Jq = rqr_deriv(q)
    Jp = rqr_deriv(p)

    check_jacobian('Jq', lambda u: rqr(u, q), numpy.zeros(3), Jq)
    check_jacobian('Jp', lambda u: rqr(u, p), numpy.zeros(3), Jp)

######################################################################

def _test_xforms():

    title('xforms')

    # a rigid transform is a (q, tvec) pair
    xa = ( quaternion_from_rvec((0.1, 0.2, 0.3)),
           numpy.array((0.55, 0.66, 0.77)) )
    
    xb = ( quaternion_from_rvec((0.11, 0.22, 0.33)),
           numpy.array((0.5, 0.6, 0.7)) )

    twist, A, B = xform_error(xa, xb, True)

    xb2 = xform_compose_twist(xa, twist)

    check('xb[0]', xb[0], 'xb2[0]', xb2[0])
    check('xb[1]', xb[1], 'xb2[1]', xb2[1])

    check_jacobian('A', lambda t: xform_error(xform_compose_twist(xa, t), xb),
                   numpy.zeros(6), A)

    check_jacobian('B', lambda t: xform_error(xa, xform_compose_twist(xb, t)),
                   numpy.zeros(6), B)

    u = numpy.array([0.1, 0.03, 0.04, 0.3, 0.6, 0.01])

    expected_pose = xform_compose_twist(xa, u)

    e, A, B = xform_error(expected_pose, xb, True)

    # A is [ de / dexpected_pose ]
    # Aprime should be [ dexpected_pose  / du ]

    #foo = num_jac( lambda u: xform_error(xform_compose_twist(xa, u), expected_pose), u )

    #printit('thingy', foo)

    xform_from_twist = lambda u: ( quaternion_from_rvec(u[:3]), u[3:] )
    (qu, tu) = xform_from_twist(u)
    R = quaternion_matrix(qu)[:3, :3]

    foo = numpy.zeros((6,6))
    foo[:3, :3] = R.T
    foo[3:, 3:] = R.T
    foo[3:, :3] = numpy.dot(R.T, -cross_matrix_3x3(tu))
    
    printit('foo', foo)
    
    thingfunc = lambda delta: xform_error(
        xform_compose_twist(xform_compose_twist(xa, delta), u), xb)
    
    A_numeric = num_jac(thingfunc, numpy.zeros(6))
    foo_actual = numpy.linalg.solve(A, A_numeric)
    
    printit('foo_actual', foo_actual)

    check('foo', foo, 'foo_actual', foo_actual)
    
    check_jacobian('A', thingfunc, numpy.zeros(6), numpy.dot(A, foo))

    sys.exit(0)


######################################################################

def _test_projections():

    title('projections')

    ##################################################
    # set up tag object points

    # tag size
    S = 0.1

    # make a bunch of points in tag coordinate frame
    object_points = numpy.array([
        [ -S,  S, 0 ],
        [  S,  S, 0 ],
        [  S, -S, 0 ],
        [ -S, -S, 0 ] ])

    printit('object_points', object_points)

    ##################################################
    # set up relative transform
    
    # assume xc = world_from_camera 
    robot_xform = ( quaternion_from_rvec((0, 1.5, 0)), numpy.array((3., 0., 2.)) )
    
    # assume tag_xform = world_from_object
    tag_xform = ( quaternion_from_rvec((0.03, 1.51, 0.01)), numpy.array((4., 0, 2.)) )

    # we want camera_from_object = inverse(robot_xform) * tag_xform
    #
    # that corresponds to the twist returned by xform_error(robot_xform, tag_xform)
    camera_from_object = xform_compose(xform_inverse(robot_xform), tag_xform)

    # get the twist (i.e. camera_from_object -> rvec, tvec)
    twist, JeA, JeB = xform_error(robot_xform, tag_xform, True)

    # make sure these correspond
    check('rvec_from_quaternion(camera_from_object[0])',
          rvec_from_quaternion(camera_from_object[0]),
          'twist[:3]', twist[:3])

    check('camera_from_object[1]',
          camera_from_object[1],
          'twist[3:]', twist[3:])
    
    ##################################################
    # transform object points into camera frame
    
    M = xform_matrix(camera_from_object)
    camera_points = numpy.dot(M[:3, :3], object_points.T).T + M[:3, 3]

    printit('M', M)
    
    printit('camera_points', camera_points)

    ##################################################
    # set up our intrinisic parameters
    
    f = 525.0

    K = numpy.array([ [ f, 0, 320.0 ],
                      [ 0, f, 240.0 ],
                      [ 0, 0, 1 ] ])

    dist_coeffs = numpy.zeros(4)

    ##################################################
    # project camera frame -> sensor plane
    
    sensor_points = numpy.dot(K, camera_points.T).T

    printit('sensor_points', sensor_points)
    
    uvs = sensor_points[:, :2] / (sensor_points[:, 2])[:, None]

    ##################################################
    # map (rvec, tvec) twist and intrinisic parameters to uvs and jacobian

    meas_func = lambda twist: cv2.projectPoints(object_points.reshape((-1, 1, 3)),
                                                twist[:3], twist[3:],
                                                K, dist_coeffs)

    uvs2, J = meas_func(twist)

    check('uvs', uvs.flatten(), 'uvs2', uvs2.flatten())

    # the first 6 columns form the array
    #
    # [ duv / dtwist ] = [ duv/drvec | duv/dtvec ]
    Jrt = J[:, 0:6]

    f = lambda twist: meas_func(twist)[0].flatten()

    check_jacobian('Jrt', f, twist, Jrt)

    ######################################################################
    
    # we want the derivative of the expected measurement with respect
    # to delta robot transform and with respect to delta tag xform
    #
    # the matrix chain rule says
    #
    #  [ duv / delta xform ] = [ duv / dtwist ] * [ dtwist / delta xform ]
    #  8x6                   = 8x6                6x6
    #
    A = numpy.dot(Jrt, JeA)
    B = numpy.dot(Jrt, JeB)

    dA = lambda delta: xform_error(xform_compose_twist(robot_xform, delta), tag_xform)
    dB = lambda delta: xform_error(robot_xform, xform_compose_twist(tag_xform, delta))

    fA = lambda delta: f(dA(delta))
    fB = lambda delta: f(dB(delta))

    check_jacobian('A', fA, numpy.zeros(6), A)
    check_jacobian('B', fB, numpy.zeros(6), B)

    print 'Holy cow, it works!'
    print
    

if __name__ == '__main__':

    numpy.set_printoptions(linewidth=200, precision=7, suppress=True)

    _test_basics()
    _test_xforms()
    _test_projections()
    

