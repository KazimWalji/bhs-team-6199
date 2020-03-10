//
//  GameScene.swift
//  BrickBeaker
//
//  Created by Kazim Walji on 3/5/20.
//  Copyright © 2020 Kazim. All rights reserved.
//

import SpriteKit
import GameplayKit

class GameScene: SKScene, SKPhysicsContactDelegate {
  var bar = SKSpriteNode()
     var ball = SKShapeNode()
    var brick = SKSpriteNode()
    var loseZone = SKSpriteNode()
    var game = false
    var xCoordinate = -200
    var yCoordinate = 60
    override func didMove(to view: SKView)
    {
           bar = self.childNode(withName: "bar") as! SKSpriteNode
        makeBall()
     makeBrick()
        makeLoseZone()
        physicsWorld.contactDelegate = self
        self.physicsBody = SKPhysicsBody(edgeLoopFrom: frame)

        
    }
    func makeBrick() {
            var brick = SKSpriteNode()
          brick = SKSpriteNode(color: .blue, size: CGSize(width: 50, height: 20))
          brick.position = CGPoint(x: (frame.midX + CGFloat(xCoordinate)), y: (frame.maxY - CGFloat(yCoordinate)))
          brick.name = "brick"
          brick.physicsBody = SKPhysicsBody(rectangleOf: brick.size)
          brick.physicsBody?.isDynamic = false
           self.addChild(brick)
   
       }
       override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
           if let touch = touches.first {
               ball.physicsBody?.isDynamic = true
                  let location = touch.location(in: self)
               bar.position.x = location.x
              }
          
           physicsWorld.contactDelegate = self
           self.physicsBody = SKPhysicsBody(edgeLoopFrom: frame)
           if(!game)
           {
           ball.physicsBody?.applyImpulse(CGVector(dx: 10, dy: 10))
    ball.physicsBody?.isDynamic = true
               game = true
           }
           
       }
    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
         if let touch = touches.first {
               let location = touch.location(in: self)
            bar.position.x = location.x
           }
        ball.physicsBody?.isDynamic = true
        physicsWorld.contactDelegate = self
        self.physicsBody = SKPhysicsBody(edgeLoopFrom: frame)
        
    }
    func makeLoseZone() {
       loseZone = SKSpriteNode(color: .red, size: CGSize(width: frame.width, height: 50))
       loseZone.position = CGPoint(x: frame.midX, y: frame.minY + 25)
       loseZone.name = "loseZone"
       loseZone.physicsBody = SKPhysicsBody(rectangleOf: loseZone.size)
       loseZone.physicsBody?.isDynamic = false
       addChild(loseZone)
    }

     override func update(_ currentTime: TimeInterval)
       {
       
        if (ball.position.y < bar.position.y - 50)
        {
            gameOver()
        }
    }
    func makeBall() {
       ball = SKShapeNode(circleOfRadius: 10)
       ball.position = CGPoint(x: frame.midX, y: frame.midY)
       ball.strokeColor = .black
       ball.fillColor = .yellow
       ball.name = "ball"

       // physics shape matches ball image
       ball.physicsBody = SKPhysicsBody(circleOfRadius: 10)
       // ignores all forces and impulses
       ball.physicsBody?.isDynamic = false
       // use precise collision detection
       ball.physicsBody?.usesPreciseCollisionDetection = true
       // no loss of energy from friction
       ball.physicsBody?.friction = 0
       // gravity is not a factor
       ball.physicsBody?.affectedByGravity = false
       // bounces fully off of other objects
       ball.physicsBody?.restitution = 1
       // does not slow down over time
       ball.physicsBody?.linearDamping = 0
       ball.physicsBody?.contactTestBitMask = (ball.physicsBody?.collisionBitMask)!
       
       addChild(ball) // add ball object to the view
    }
    
   
    func gameOver()
    {

        game = false
        ball.physicsBody?.isDynamic = false
         ball.position = CGPoint(x: frame.midX, y: frame.midY)
    }
    func didBegin(_ contact: SKPhysicsContact) {
       if contact.bodyA.node?.name == "brick" ||
          contact.bodyB.node?.name == "brick" {
             print("You win!")
             brick.removeFromParent()
       }
       if contact.bodyA.node?.name == "loseZone" ||
          contact.bodyB.node?.name == "loseZone" {
             print("You lose!")
             ball.removeFromParent()
       }
    }
}
