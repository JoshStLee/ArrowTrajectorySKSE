# CommonLibSSE-NG Empty Plugin Template

A minimal SKSE plugin template for Skyrim SE/AE using CommonLibSSE-NG. It does not include PrismaUI or enable Skyrim VR.

# Current feature milestone

While the player is actively drawing/holding a bow or crossbow attack with ammunition equipped, the plugin predicts the arrow's ballistic path, draws it as a thick white Direct3D overlay, and moves the vanilla HUD crosshair to the predicted impact point when the path hits something. Combat spells use a straight camera ray and move the same HUD crosshair to the raycast hit.

This milestone uses the vanilla HUD reticle. SmoothCam's selectable 3D crosshair styles are not included yet.

# Requires ClibDT for the sake of your glutes
# Getting Started
ClibDT (https://www.nexusmods.com/skyrimspecialedition/mods/154240)

if you botch the installation you're gay and this entire repo won't do you any good

# Build
if you actually nail the installation, a simple `xmake build` would suffice
