using Base:Float32, aligned_sizeof
using GLMakie

GLMakie.activate!()

##

# Define constants and types

#Boid Settings
NUM_BOIDS = 200
VELOCITY = 150
VIEW_DISTANCE = 100
VIEW_ANGLE = 4π/3

# Boid Modifiers

ALIGNMENT_MODIFIER = .3
COHESION_MODIFIER = .5
SEPERATION_MODIFIER = .2
OBSTACLE_MODIFIER = .5

# Animation settings
FRAMERATE = 30
SECONDS = 120

# Plot Settings
X_WIDTH = 2000
Y_WIDTH = 1000

mutable struct Boids
    xs::Vector{Float64} # m
    ys::Vector{Float64} # m
    θ::Vector{Float64} # radians, 0 = right (unit circle)
end

Boids() = Boids(rand(NUM_BOIDS) * X_WIDTH, rand(NUM_BOIDS) * Y_WIDTH, rand(NUM_BOIDS) * 2π)

##

function iterateBoids!(boidsArr)
    # Moves based on previous heading
    boidsArr.xs = boidsArr.xs .+ (VELOCITY / FRAMERATE) * cos.(boidsArr.θ) 
    boidsArr.ys = boidsArr.ys .+ (VELOCITY / FRAMERATE) * sin.(boidsArr.θ) 

    boidsArr.xs = map(loopX!, boidsArr.xs)
    boidsArr.ys = map(loopY!, boidsArr.ys)


    for i in 1:NUM_BOIDS

        num_in_range = 0
        θttl = 0
        xttl = 0
        yttl = 0
        θtottl = 0

        dist = .√((boidsArr.xs .- boidsArr.xs[i]) .^ 2 .+ (boidsArr.ys .- boidsArr.ys[i]) .^ 2)
        θto = atan.((boidsArr.xs .- boidsArr.xs[i]) ./ (boidsArr.ys .- boidsArr.ys[i])) .- boidsArr.θ[i] # Temporarily ignore case of being the same pt or 'inline`

        for inner in 1:NUM_BOIDS
            if dist[inner] != 0 && dist[inner] < VIEW_DISTANCE && abs(θto[inner]) < VIEW_ANGLE
                num_in_range += 1
                θttl += boidsArr.θ[inner]
                xttl += boidsArr.xs[inner]
                yttl += boidsArr.ys[inner]
                θtottl += θto[inner]
            end
        end

        if X_WIDTH - boidsArr.xs[i] < VIEW_DISTANCE
            xWallNudge = (π - boidsArr.θ[i]) * OBSTACLE_MODIFIER * X_WIDTH - boidsArr.xs[i]
        elseif boidsArr.xs[i] < VIEW_DISTANCE
            xWallNudge = (0 - boidsArr.θ[i]) * OBSTACLE_MODIFIER * boidsArr.xs[i]
        else
            xWallNudge = 0
        end

        if Y_WIDTH - boidsArr.ys[i] < VIEW_DISTANCE
            yWallNudge = (3π/2 - boidsArr.θ[i]) * OBSTACLE_MODIFIER * Y_WIDTH - boidsArr.ys[i]
        elseif boidsArr.ys[i] < VIEW_DISTANCE
            yWallNudge = (π/2 - boidsArr.θ[i]) * OBSTACLE_MODIFIER * boidsArr.ys[i]
        else
            yWallNudge = 0
        end

        if (num_in_range > 0)
            θavg = θttl / num_in_range
            xavg = xttl / num_in_range
            yavg = yttl / num_in_range
            θtoavg = θtottl / num_in_range

            #ALIGNMENT
            alignmentNudge = (θavg - boidsArr.θ[i])*ALIGNMENT_MODIFIER

            #COHESION
            angleToCohesion = atan(xavg - boidsArr.xs[i] / yavg - boidsArr.ys[i])
            cohesionNudge = (angleToCohesion - boidsArr.θ[i])*COHESION_MODIFIER
            
            #SEPERATION
            angleToSeperation = θtoavg - π
            if angleToSeperation > 2π
                angleToSeperation -= 2π
            elseif angleToSeperation < 0
                angleToSeperation += 2π
            end
            seperationNudge = angleToSeperation - boidsArr.θ[i]*SEPERATION_MODIFIER
            
            totalNudge = cohesionNudge + alignmentNudge + seperationNudge + xWallNudge + yWallNudge
        else
            totalNudge = rand()*0.1 - .05
        end

        boidsArr.θ[i] += totalNudge / FRAMERATE
    end

    return boidsArr
end

function loopX!(xval)
    if xval > X_WIDTH
        xval = 0
    elseif xval < 0
        xval = X_WIDTH
    end
    return xval
end
function loopY!(yval)
    if yval > Y_WIDTH
        yval = 0
    elseif yval < 0
        yval = Y_WIDTH
    end
    return yval
end
##
# Define Figure

fig = Figure(resolution=(X_WIDTH, Y_WIDTH))

ax = fig[1, 1] = Axis(fig)

hidespines!(ax)
hidedecorations!(ax)

limits!(ax, 0, X_WIDTH, 0, Y_WIDTH)

##
boids = Boids()

timestamps = range(0, SECONDS, step=1 / FRAMERATE)

record(fig, "randimation.mp4", timestamps; framerate = FRAMERATE) do i
    empty!(ax)


    iterateBoids!(boids)
    # tans = atan.(boids.yv ./ boids.xv)
    scatter!(ax, boids.xs, boids.ys, color = :black)
end

##" 