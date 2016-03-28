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

    q, t = xform
    rvec = twist[0:3]
    tvec = twist[3:6]
    qnew = quaternion_multiply(quaternion_from_rvec(rvec), q)
    tnew = t + tvec
    return (qnew, tnew)

######################################################################
# we want xform_compose_twist(xb, xform_error(xa, xb)) == xa

def xform_error(xform_a, xform_b):

    # simple way
    qa, ta = xform_a
    qb, tb = xform_b
    q = quaternion_multiply(qa, quaternion_inverse(qb))
    twist = numpy.zeros(6)
    twist[0:3] = rvec_from_quaternion(q)
    twist[3:6] = ta - tb
    return twist

######################################################################
# returns the jacobian of xform_error with respect to a twist applied
# to A, evaluated at twist = 0

def xform_error_jacobian(xform_a, xform_b):

    qa, ta = xform_a
    qb, tb = xform_b
    q = quaternion_multiply(qb, quaternion_inverse(qa))

    A = numpy.zeros((6,6))

    A[:3,:3] = rqr_deriv(q).T
    A[3:,3:] = numpy.eye(3)

    return A

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
    xb = ( quaternion_from_rvec((0.3, 0.1, 0.2)),
           numpy.array((0.5, 0.6, 0.7)) )

    # a "twist" is a differential rotation vector, and translation
    # vector, concatenated to be 6x1 (e.g. an "action")
    twist = numpy.array([ 0.05, 0.03, 0.02, 0.04, 0.07, 0.01 ] )

    # composing twists with transforms acts on the transform's
    # rotation and translation independently
    xa = xform_compose_twist(xb, twist)

    # xform_error(xa, xb) returns the twist to compose with xb in
    # order to obtain xa.
    #
    # hence, we expect that
    #
    #   xform_compose_twist(xb, xform_error(xa, xb)) == xa
    twist2 = xform_error(xa, xb)
    
    check('twist', twist, 'twist2', twist2)
    
    xa2 = xform_compose_twist(xb, xform_error(xa, xb))

    check('xa[0]', xa[0], 'xa2[0]', xa2[0])
    check('xa[1]', xa[1], 'xa2[1]', xa2[1])

    # make sure our jacobians work ok
    A = xform_error_jacobian(xa, xb)
    B = -A.T

    check_jacobian('A', lambda t: xform_error(xform_compose_twist(xa, t), xb),
                   numpy.zeros(6), A)

    check_jacobian('B', lambda t: xform_error(xa, xform_compose_twist(xb, t)),
                   numpy.zeros(6), B)

######################################################################

def _test_projections():

    title('projections')

    # tag size
    S = 0.03

    object_points = numpy.array([
        [ -S,  S, 0 ],
        [  S,  S, 0 ],
        [  S, -S, 0 ],
        [ -S, -S, 0 ] ])

    # assume xc = world_from_camera 
    # assume xo = world_from_object
    # we want camera_from_object = inverse(xc) * xo

    # our goal is ACTUALLY to get Jacobians of uv's in image wrt twists applied to xc and xo

    xc = ( quaternion_from_rvec((0, 1.5, 0)), numpy.array((3., 0., 2.)) )
    xo = ( quaternion_from_rvec((0, 1.51, 0)), numpy.array((4., 0, 2.)) )

    rel_xform = lambda xc, xo: xform_compose( xform_inverse(xc), xo )
    rvec_func = lambda xc, xo: rvec_from_quaternion(rel_xform(xc, xo)[0])
    tvec_func = lambda xc, xo: rel_xform(xc, xo)[1]

    rvec = rvec_func(xc, xo)
    tvec = tvec_func(xc, xo)

    # let's look at JArvec = [ drvec / dxc ]
    JArvec_num = num_jac(lambda delta: rvec_func(xform_compose_twist(xc, delta), xo),
                         numpy.zeros(6))

    JAtvec_num = num_jac(lambda delta: tvec_func(xform_compose_twist(xc, delta), xo),
                         numpy.zeros(6))

    # let's look at JBrvec = [ drvec / dxc ]
    JBrvec_num = num_jac(lambda delta: rvec_func(xc, xform_compose_twist(xo, delta)),
                         numpy.zeros(6))

    JBtvec_num = num_jac(lambda delta: tvec_func(xc, xform_compose_twist(xo, delta)),
                         numpy.zeros(6))

    printit('JArvec_num', JArvec_num)
    printit('JAtvec_num', JAtvec_num)

    printit('JBrvec_num', JBrvec_num)
    printit('JBtvec_num', JBtvec_num)
    
    sys.exit(0)

    M = xform_matrix(rel_xform(xc, xo))

    camera_points = numpy.dot(M[:3, :3], object_points.T).T + tvec

    f = 525.0

    p = numpy.array([f, f, 320.0, 240.0])
    
    make_k = lambda p: numpy.array([[p[0], 0, p[2]],
                                    [0, p[1], p[3]],
                                    [0, 0, 1]])

    K = make_k(p)

    dist_coeffs = numpy.zeros(4)

    sensor_points = numpy.dot(K, camera_points.T).T

    uvs = sensor_points[:, :2] / (sensor_points[:, 2])[:, None]

    #cv2.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs[, imagePoints[, jacobian[, aspectRatio]]])

    # let N be our number of points
    # J is 2N (number of output coords) by 10 + num dist coeffs
    uvs2, J = cv2.projectPoints(object_points.reshape((-1, 1, 3)),
                                rvec, tvec,
                                K, dist_coeffs)

    Jrvec = J[:, 0:3]
    Jtvec = J[:, 3:6]
    JK = J[:, 6:10]

    printit('M', M)
    printit('object_points', object_points)
    printit('camera_points', camera_points)
    printit('sensor_points', sensor_points)
    check('uvs', uvs, 'uvs2', uvs2.reshape((-1, 2)))

    check_jacobian('Jrvec',
                   lambda r: cv2.projectPoints(object_points.reshape((-1, 1, 3)),
                                               r, tvec, K, dist_coeffs)[0].flatten(),
                   rvec,
                   Jrvec)

    check_jacobian('Jtvec',
                   lambda t: cv2.projectPoints(object_points.reshape((-1, 1, 3)),
                                               rvec, t, K, dist_coeffs)[0].flatten(),
                   tvec,
                   Jtvec)

    check_jacobian('JK',
                   lambda p: cv2.projectPoints(object_points.reshape((-1, 1, 3)),
                                               rvec, tvec, make_k(p), dist_coeffs)[0].flatten(),
                   p,
                   JK)

    # eventually, let A be [ duv / twist applied to camera ] 8x6
    # eventually, let B be [ duv / twist applied to object ] 8x6

    # based on the matrix chain rule:
    #
    # total derivative = sum of partials
    #
    # A = [duv/dtvec] * [dtvec/ twist to camera ] + [ duv/drvec ] * [ drvec / twist to camera ]
    #   = 8x3         * 3x6                         8x3             3x6
    
if __name__ == '__main__':

    numpy.set_printoptions(linewidth=200, precision=7, suppress=True)

    _test_basics()
    _test_xforms()
    _test_projections()
    

