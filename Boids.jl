using GLMakie
using BenchmarkTools
GLMakie.activate!()

##

NUM_BOIDS = 100
VIEW_DISTANCE = 70 
VIEW_ANGLE = 4π/3
SIDE_MARGIN = 50
MARGIN_TURN = 1

# Boid Modifiers

ALIGNMENT_MODIFIER = 0.1
COHESION_MODIFIER = 0.1
SEPERATION_MODIFIER = 0.02
MIN_DISTANCE = VIEW_DISTANCE / 10

MAX_VELOCITY = 120
MIN_VELOCITY = 10
MAX_ACCELERATION = 10

# Animation settings
FRAMERATE = 30
SECONDS = 30

# Plot Settings
X_WIDTH = 1000
Y_WIDTH = 1000

mutable struct Boids
    x::Vector{Float64} # m
    y::Vector{Float64} # m
    dx::Vector{Float64} # m/s
    dy::Vector{Float64} # m/s
end

Boids(x) = Boids(rand(x) * X_WIDTH, rand(x) * Y_WIDTH, rand(x)*(MAX_VELOCITY) .- MIN_VELOCITY, rand(x)*(MAX_VELOCITY) .- MIN_VELOCITY)

##


function iterateBoids!(boidsArr)
    # Iterate positions
    
    for i in 1:NUM_BOIDS
        dist = .√(((boidsArr.x .- boidsArr.x[i]) .^ 2) .+ ((boidsArr.y .- boidsArr.y[i]) .^ 2))

        xs = []
        ys = []
        dxttl = 0
        dyttl = 0
        moveX = 0
        moveY = 0
        numInRange = 0

        for inner in 1:NUM_BOIDS
            if dist[inner] > 0.1 && dist[inner] < VIEW_DISTANCE
                #Cohesion
                push!(ys, boidsArr.y[inner])
                push!(xs, boidsArr.x[inner])

                dxttl += boidsArr.dx[inner]
                dyttl += boidsArr.dy[inner]
                #Seperation
                moveX += boidsArr.x[i] - boidsArr.x[inner]
                moveY += boidsArr.y[i] - boidsArr.y[inner]

                numInRange += 1
            end

            if numInRange > 0
                # COHESION
                avgX = sum(xs) / numInRange
                avgY = sum(ys) / numInRange
                
                boidsArr.dx[i] += (avgX - boidsArr.x[i]) * COHESION_MODIFIER
                boidsArr.dy[i] += (avgY - boidsArr.y[i]) * COHESION_MODIFIER

                #SEPERATION
                boidsArr.dx[i] += moveX * SEPERATION_MODIFIER
                boidsArr.dy[i] += moveY * SEPERATION_MODIFIER

                #ALIGNMENT
                avgDX = dxttl / numInRange
                avgDY = dyttl / numInRange

                boidsArr.dx[i] += (avgDX - boidsArr.dx[i]) * ALIGNMENT_MODIFIER
                boidsArr.dy[i] += (avgDY - boidsArr.dy[i]) * ALIGNMENT_MODIFIER
            end
            
            #Limit Velocity
            mag = getMagnitude(boidsArr.dx[i], boidsArr.dy[i])
            if mag > MAX_VELOCITY
                boidsArr.dx[i] = (boidsArr.dx[i] / mag) * MAX_VELOCITY
                boidsArr.dy[i] = (boidsArr.dy[i] / mag) * MAX_VELOCITY
            end

            #Stay in bounds
            if boidsArr.x[i] < SIDE_MARGIN
                boidsArr.dx[i] += MARGIN_TURN
            elseif boidsArr.x[i] > X_WIDTH - SIDE_MARGIN
                boidsArr.dx[i] -= MARGIN_TURN
            end
            if boidsArr.y[i] < SIDE_MARGIN
                boidsArr.dy[i] += MARGIN_TURN
            elseif boidsArr.y[i] > Y_WIDTH - SIDE_MARGIN
                boidsArr.dy[i] -= MARGIN_TURN
            end
        end
    end


    boidsArr.x = boidsArr.x .+ (boidsArr.dx ./ FRAMERATE)
    boidsArr.y = boidsArr.y .+ (boidsArr.dy ./ FRAMERATE)
end


function getMagnitude(dx, dy)
    return √(dx^2 + dy^2)
end

##

# Define Figure

fig = Figure(resolution=(X_WIDTH, Y_WIDTH))

ax = fig[1, 1] = Axis(fig)

hidespines!(ax)
hidedecorations!(ax)

limits!(ax, 0, X_WIDTH, 0, Y_WIDTH)

##
boids = Boids(NUM_BOIDS)

timestamps = range(0, SECONDS, step=1 / FRAMERATE)

record(fig, "animation.mp4", timestamps; framerate = FRAMERATE) do i
    empty!(ax)
    iterateBoids!(boids)
    # tans = atan.(boids.yv ./ boids.xv)
    scatter!(ax, boids.x, boids.y, color = :black)
end

##

@benchmark iterateBoids!(b) setup=(b = Boids(NUM_BOIDS)) seconds=10

